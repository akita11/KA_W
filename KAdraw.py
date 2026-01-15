import serial
import serial.tools.list_ports as list_ports
import matplotlib
import sys
import os
import csv
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox, CheckButtons
from matplotlib.animation import FuncAnimation

# Detect headless mode: CLI flag `--headless`, env `HEADLESS`, or non-interactive backend
_headless = False
try:
	if '--headless' in sys.argv:
		_headless = True
	elif os.environ.get('HEADLESS', '').lower() in ('1', 'true', 'yes'):
		_headless = True
	else:
		try:
			backend = matplotlib.get_backend().lower()
			if 'agg' in backend and sys.platform != 'win32':
				_headless = True
		except Exception:
			pass
except Exception:
	_headless = False

#SERIAL_PORT = 'COM125'
SERIAL_PORT = '/dev/tty.usbmodem101'
# 0度と90度での1,2のDCMを基準として、膝角度（ねじりを含む）を求める
# ←IMUの設置向きから、e1,e2,e3のどれを使うかを確認
# STARTからの同期

# 3次元姿勢は一義的にはクオータニオン
# それをオイラー角（さらにはそこからDCM）に変換するときに「おかしく見える」ケースが出てくる（特異点など）
# クオータニオンから直接DCMでいけるはず
# MF内はクオータニオン→https://lipoyang.hatenablog.com/entry/2020/03/27/202037
#2つのIMUの片方を基準（例えばe1=(1,0,0)）となるように回転させれば、2つのIMUのなす角を求められる。
#そのうち、膝関節の稼働方向に最も近い向きの角度（例えば2つのe1がなす角）を求めればよさそう。

# クオータニオン・オイラー角・DCM
# https://qiita.com/drken/items/0639cf34cce14e8d58a5
# https://rikei-tawamure.com/entry/2025/03/07/080933
# https://rikei-tawamure.com/entry/2025/03/16/131438
# https://rikei-tawamure.com/entry/2025/03/15/225248
# https://www.sbg-systems.com/ja/glossary/attitude-in-navigation/

# --- 設定パラメータ ---
BAUD_RATE = 115200
MAX_SAMPLES = 200     # グラフに表示する最大サンプル数
N_DRAW = 10           # 描画更新の間隔（サンプル数）

# --- データ格納用バッファ (リングバッファ) ---
t_data = deque(maxlen=MAX_SAMPLES)
# buffer for calibrated Theta and raw Theta history
theta_calib_buffer = deque(maxlen=MAX_SAMPLES)
theta_raw_buffer = deque(maxlen=MAX_SAMPLES)
# buffers for individual sensor pitches (degrees)
pitch1_buffer = deque(maxlen=MAX_SAMPLES)
pitch2_buffer = deque(maxlen=MAX_SAMPLES)

# capture state for Get0/Get90: collect N samples after button press
CAPTURE_SAMPLES = 20
capture_mode = None  # '0' or '90' when capturing
capture_buffer = []

current_t = 0.0
sample_count = 0

# calibration state
calib_mode = False
calib_data = []
Drel0 = np.eye(3)
# last raw quaternions (w,x,y,z) for new serial format (kept for handlers if needed)
last_raw_q1 = [1.0, 0.0, 0.0, 0.0]
last_raw_q2 = [1.0, 0.0, 0.0, 0.0]

# シリアルポートの設定（起動時は未接続、UIからOpen/Closeで操作）
ser = None
available_ports = []
selected_port = None
selected_index = 0
# saved DCMs for calibration (0deg and 90deg) for each sensor

# draw flags for basis vectors in Fig3 (default: only e3 True)
draw_e1 = False
draw_e2 = False
draw_e3 = True
# Show/hide Fig3 window
show_basis = False
# Theta calibration points (degrees) - default 0 and 90
theta0 = 0.0
theta90 = 90.0

def circular_mean_deg(values):
	"""Compute circular mean of angles in degrees. Returns value in (-180,180]."""
	if not values:
		return 0.0
	a = np.deg2rad(np.array(values))
	s = np.sum(np.sin(a))
	c = np.sum(np.cos(a))
	ang = np.arctan2(s, c)
	deg = np.rad2deg(ang)
	# normalize to (-180,180]
	deg = (deg + 180.0) % 360.0 - 180.0
	return deg

def angdiff(a, b):
	"""Return minimal difference a - b in degrees normalized to (-180,180]."""
	d = (a - b + 180.0) % 360.0 - 180.0
	return d

def quat_conjugate(q):
	"""Return conjugate of quaternion q = [w,x,y,z]."""
	return [q[0], -q[1], -q[2], -q[3]]

def quat_mul(a, b):
	"""Quaternion multiplication (Hamilton product): a*b for [w,x,y,z]."""
	aw, ax, ay, az = a
	bw, bx, by, bz = b
	w = aw*bw - ax*bx - ay*by - az*bz
	x = aw*bx + ax*bw + ay*bz - az*by
	y = aw*by - ax*bz + ay*bw + az*bx
	z = aw*bz + ax*by - ay*bx + az*bw
	return [w, x, y, z]

def quat_to_ypr(q):
	"""Convert quaternion [w,x,y,z] to yaw,pitch,roll (radians) using standard Tait-Bryan Z-Y-X convention.
	Returns (yaw, pitch, roll).
	"""
	w, x, y, z = q
	# roll (x-axis rotation)
	t0 = 2.0*(w*x + y*z)
	t1 = 1.0 - 2.0*(x*x + y*y)
	roll = np.arctan2(t0, t1)
	# pitch (y-axis rotation)
	t2 = 2.0*(w*y - z*x)
	t2 = np.clip(t2, -1.0, 1.0)
	pitch = np.arcsin(t2)
	# yaw (z-axis rotation)
	t3 = 2.0*(w*z + x*y)
	t4 = 1.0 - 2.0*(y*y + z*z)
	yaw = np.arctan2(t3, t4)
	return yaw, pitch, roll

# Angle history buffers for basis comparisons
angle_e1_set1 = deque(maxlen=MAX_SAMPLES)
angle_e2_set1 = deque(maxlen=MAX_SAMPLES)
angle_e3_set1 = deque(maxlen=MAX_SAMPLES)
angle_e1_set2 = deque(maxlen=MAX_SAMPLES)
angle_e2_set2 = deque(maxlen=MAX_SAMPLES)
angle_e3_set2 = deque(maxlen=MAX_SAMPLES)

def scan_ports():
	"""Populate available_ports list with device names."""
	global available_ports, selected_port
	try:
		ports = list_ports.comports()
		available_ports = [p.device for p in ports]
		if len(available_ports) > 0:
			selected_port = available_ports[0]
		else:
			selected_port = None
	except Exception:
		available_ports = []
		selected_port = None

# グラフの設定 - 単一描画軸 (pitch3_deg と補正後のみ)
fig, ax_main = plt.subplots(1, 1, figsize=(12, 6))
# backward-compatible names (some code may reference these)
ax_theta = ax_main
ax_pitchdiff = ax_main
ax_roll = ax_main
ax_pitch = ax_main
ax_yaw = ax_main

# fig3 (3D basis window) will be created on demand when requested
fig3 = None
ax3_set1 = None
ax3_set2 = None

# fig2: IMU pose visualization (two 3D axes for Q1 and Q2)
fig2 = None
ax2_q1 = None
ax2_q2 = None

def quat_to_dcm(q):
	"""Convert quaternion [w,x,y,z] to direction cosine matrix (3x3).
	Returns numpy array shape (3,3).
	"""
	w, x, y, z = q
	# normalize just in case
	n = w*w + x*x + y*y + z*z
	if n == 0:
		return np.eye(3)
	s = 1.0 / n
	ww = w*w*s
	xx = x*x*s
	yy = y*y*s
	zz = z*z*s
	wx = w*x*s
	wy = w*y*s
	wz = w*z*s
	xy = x*y*s
	xz = x*z*s
	yz = y*z*s
	R = np.array([
		[1.0 - 2.0*(yy + zz),     2.0*(xy - wz),         2.0*(xz + wy)],
		[2.0*(xy + wz),           1.0 - 2.0*(xx + zz),   2.0*(yz - wx)],
		[2.0*(xz - wy),           2.0*(yz + wx),         1.0 - 2.0*(xx + yy)]
	])
	return R

def _create_fig2():
	"""Create Figure2 with two 3D axes to show IMU orientations from Q1 and Q2."""
	global fig2, ax2_q1, ax2_q2
	try:
		fig2_local = plt.figure(num=2, figsize=(8, 4))
		ax_list = fig2_local.subplots(1, 2, subplot_kw={'projection': '3d'})
		fig2 = fig2_local
		ax2_q1, ax2_q2 = ax_list[0], ax_list[1]
		ax2_q1.set_title('IMU1 (Q1)')
		ax2_q2.set_title('IMU2 (Q2)')
		for a in (ax2_q1, ax2_q2):
			a.set_xlim(-1.1, 1.1)
			a.set_ylim(-1.1, 1.1)
			a.set_zlim(-1.1, 1.1)
			a.set_xlabel('X')
			a.set_ylabel('Y')
			a.set_zlabel('Z')
	except Exception:
		fig2 = None
		ax2_q1 = None
		ax2_q2 = None

def _create_fig3():
	"""Create the fig3 window and axes for basis visualization."""
	global fig3, ax3_set1, ax3_set2
	try:
		fig3_local, axes3 = plt.subplots(1, 2, subplot_kw={'projection': '3d'}, figsize=(8, 4))
		fig3 = fig3_local
		ax3_set1 = axes3[0]
		ax3_set2 = axes3[1]
		ax3_set1.set_title('Set1 basis (e1,e2,e3)')
		ax3_set2.set_title('Set2 basis (e1,e2,e3)')
		for ax3 in (ax3_set1, ax3_set2):
			ax3.set_xlim(-1.1, 1.1)
			ax3.set_ylim(-1.1, 1.1)
			ax3.set_zlim(-1.1, 1.1)
			ax3.set_xlabel('X')
			ax3.set_ylabel('Y')
			ax3.set_zlabel('Z')
		# default to showing all basis vectors when fig3 is created
		try:
			global draw_e1, draw_e2, draw_e3
			draw_e1 = True
			draw_e2 = True
			draw_e3 = True
		except Exception:
			pass

		# CheckButtons on fig3 to toggle e1/e2/e3 drawing
		try:
			cb_ax = fig3.add_axes([0.02, 0.02, 0.12, 0.12])
			cb = CheckButtons(cb_ax, ['e1', 'e2', 'e3'], [draw_e1, draw_e2, draw_e3])
			def _on_check(label):
				global draw_e1, draw_e2, draw_e3, fig3, fig2
				if label == 'e1':
					draw_e1 = not draw_e1
				elif label == 'e2':
					draw_e2 = not draw_e2
				elif label == 'e3':
					draw_e3 = not draw_e3
				# request redraw of fig3 and fig2 so visual updates occur immediately
				try:
					if fig3 is not None:
						fig3.canvas.draw_idle()
					if fig2 is not None:
						fig2.canvas.draw_idle()
					# small pause to ensure window refresh on some backends
					plt.pause(0.001)
				except Exception:
					pass
			cb.on_clicked(_on_check)
		except Exception:
			pass
	except Exception:
		fig3 = None
		ax3_set1 = None
		ax3_set2 = None

# UI state
dataset_mode = 'Both'   # 'Set1','Set2','Both' (デフォルトで両方表示)

# CSV recording state
recording = False
csv_filename = 'data.csv'
csv_fh = None
csv_writer = None
# Save timing for CSV: start at 0 when saving begins, step by SAVE_STEP (4ms)
SAVE_STEP = 0.004
save_time = 0.0

def euler_to_dcm(yaw_deg, roll_deg, pitch_deg):
	"""Yaw(Z), Roll(X), Pitch(Y) -> DCM using Z-Y-X (yaw-pitch-roll) sequence.
	Inputs can be scalars or numpy arrays in degrees. Returns array (...,3,3).
	"""
	y = np.deg2rad(yaw_deg)
	r = np.deg2rad(roll_deg)
	p = np.deg2rad(pitch_deg)

	cy = np.cos(y); sy = np.sin(y)
	cr = np.cos(r); sr = np.sin(r)
	cp = np.cos(p); sp = np.sin(p)

	# R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
	d00 = cy * cp
	d01 = cy * sp * sr - sy * cr
	d02 = cy * sp * cr + sy * sr

	d10 = sy * cp
	d11 = sy * sp * sr + cy * cr
	d12 = sy * sp * cr - cy * sr

	d20 = -sp
	d21 = cp * sr
	d22 = cp * cr

	# scalar inputs -> return (3,3), array inputs -> return (N,3,3)
	if np.isscalar(yaw_deg):
		return np.array([[d00, d01, d02], [d10, d11, d12], [d20, d21, d22]])
	else:
		stacked = np.stack([d00, d01, d02, d10, d11, d12, d20, d21, d22], axis=-1)
		return stacked.reshape(-1, 3, 3)

def init():
	# initialize single main axis
	try:
		ax_main.set_xlim(0, 5)
		ax_main.set_ylim(-180, 180)
		ax_main.grid(True)
		ax_main.set_xlabel('Time [s]')
	except Exception:
		pass
	return []

def update(frame):
	global current_t, sample_count, dataset_mode
	global recording, csv_filename, csv_fh, csv_writer
	global calib_mode, calib_data, Drel0
	# globals for last quaternions and buffers
	global last_raw_q1, last_raw_q2, theta_raw_buffer, theta_calib_buffer
	# save timing
	global save_time, SAVE_STEP
	# capture state
	global capture_mode, capture_buffer, CAPTURE_SAMPLES
	# theta calibration points
	global theta0, theta90

	# read all available serial lines (only if serial is open)
	if ser is not None:
		while ser.in_waiting > 0:
			line_raw = ser.readline().decode('utf-8').strip()
			if not line_raw:
				continue
			# Debug: show raw received line
			#print('RX:', line_raw)
			#try:
			#	print('RX:', line_raw)
			#except Exception:
			#	pass
			# parse and process a serial line (quaternion format expected)
			try:
				parts = line_raw.split(',')
				# Support new quaternion format: dt, q1(w,x,y,z), q2(w,x,y,z)
				if len(parts) < 10:
					# non-quaternion line — skip
					continue
				# parse values
				dt_val = float(parts[1])
				q1 = [float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])]
				q2 = [float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9])]
				# save last raw quaternions for capture/handlers
				last_raw_q1 = q1
				last_raw_q2 = q2
				# compute relative rotation Q3 = q1^{-1} * q2
				q1_conj = quat_conjugate(q1)
				q3 = quat_mul(q1_conj, q2)
				# get yaw,pitch,roll from Q3 (radians)
				yaw3, pitch3, roll3 = quat_to_ypr(q3)
				pitch3_deg = float(np.rad2deg(pitch3))
				# Theta (raw) = Y-axis rotation (pitch) from Q3, normalized to (-180,180]
				theta_raw = angdiff(pitch3_deg, 0.0)
				# also compute individual sensor pitches from Q1 and Q2
				yaw1, pitch1, roll1 = quat_to_ypr(q1)
				yaw2, pitch2, roll2 = quat_to_ypr(q2)
				pitch1_deg = angdiff(float(np.rad2deg(pitch1)), 0.0)
				pitch2_deg = angdiff(float(np.rad2deg(pitch2)), 0.0)
				# append sensor pitches
				try:
					pitch1_buffer.append(pitch1_deg)
				except Exception:
					pass
				try:
					pitch2_buffer.append(pitch2_deg)
				except Exception:
					pass
				# store immediately so current sample is available
				try:
					theta_raw_buffer.append(theta_raw)
				except Exception:
					pass
				# make current value available for later processing
				current_theta_raw = theta_raw
				# if we're in capture mode, collect samples (pitch only) and finish when enough
				if capture_mode is not None:
					try:
						# store normalized raw theta for capture averaging
						capture_buffer.append(angdiff(pitch3_deg, 0.0))
					except Exception:
						pass
					if len(capture_buffer) >= CAPTURE_SAMPLES:
						mean_theta_raw = circular_mean_deg(capture_buffer) if capture_buffer else pitch3_deg
						try:
							_compute_and_store_dcms(capture_mode, mean_override=mean_theta_raw)
						except Exception:
							pass
						try:
							update_button_states()
						except Exception:
							pass
						capture_mode = None
						capture_buffer = []

				# common post-parse processing: increment time, compute Theta and CSV write
				current_t += dt_val
				t_data.append(current_t)
				# count samples for drawing throttle
				sample_count += 1
				# Compute Theta from raw pitch-difference using theta0/theta90 mapping
				# prefer Y-rotation from quaternion Q3 if available
				# current_theta_raw already set when parsing; fallback if not
				if 'current_theta_raw' not in locals():
					if len(theta_raw_buffer) > 0:
						current_theta_raw = float(theta_raw_buffer[-1])
					else:
						current_theta_raw = 0.0
				# compute calibrated Theta (linear mapping so that Theta0->0, Theta90->90)
				if theta0 is not None and theta90 is not None:
					delta_cal = angdiff(theta90, theta0)
					if abs(delta_cal) < 1e-6:
						theta_calib = 0.0
					else:
						slope = 90.0 / delta_cal
						theta_calib = slope * angdiff(current_theta_raw, theta0)
					# normalize to [-180,180)
					if theta_calib >= 180.0:
						theta_calib -= 360.0
					elif theta_calib < -180.0:
						theta_calib += 360.0
				else:
					theta_calib = 0.0
				# Debug: show theta values for this sample
				try:
					print('theta_raw=', f"{theta_raw:.3f}", 'theta_calib=', f"{theta_calib:.3f}", 'theta0=', theta0, 'theta90=', theta90)
				except Exception:
					pass
				# store calibrated history (raw already appended)
				theta_calib_buffer.append(theta_calib)

				# CSV書き出し: recording が有効なら 1 行にまとめて保存
				if recording and csv_writer is not None:
					# write as: t[s], theta_calib, theta_raw, q1_w,q1_x,q1_y,q1_z, q2_w,q2_x,q2_y,q2_z
					row = [f"{save_time:.3f}", f"{theta_calib:.3f}", f"{current_theta_raw:.3f}"]
					if last_raw_q1 is not None:
						row += [f"{v:.6f}" for v in last_raw_q1]
					else:
						row += ['', '', '', '']
					if last_raw_q2 is not None:
						row += [f"{v:.6f}" for v in last_raw_q2]
					else:
						row += ['', '', '', '']
					try:
						csv_writer.writerow(row)
					except Exception:
						# failed to write this row; ignore for now
						pass
					try:
						csv_fh.flush()
					except Exception:
						pass
					# advance save_time even if flush fails
					try:
						save_time += SAVE_STEP
					except Exception:
						pass
			except Exception:
				# parsing/processing error for this line; skip
				pass

		if sample_count >= N_DRAW and len(t_data) > 0:
			sample_count = 0

			# 単一軸をクリアして描画
			ax_main.cla()
			ax_main.grid(True)
			ax_main.set_xlabel('Time [s]')
			ax_main.set_ylabel('Angle [deg]')
			# keep y-range fixed to [-180,180]
			ax_main.set_ylim(-180, 180)

			times = np.array(t_data)

			# raw pitch (Q3), calibrated theta, and sensor pitches
			theta_raw_arr = np.array(theta_raw_buffer)
			theta_calib_arr = np.array(theta_calib_buffer)
			pitch1_arr = np.array(pitch1_buffer)
			pitch2_arr = np.array(pitch2_buffer)

			# plot raw pitch (Q3)
			if len(theta_raw_arr) > 0 and len(times) > 0:
				if len(theta_raw_arr) == len(times):
					t_plot = times
				else:
					# align tail
					t_plot = times[-len(theta_raw_arr):]
				ax_main.plot(t_plot, theta_raw_arr, label='Theta (raw) [deg]', color='blue', linestyle=':')

			# plot sensor pitches
			if len(pitch1_arr) > 0 and len(times) > 0:
				if len(pitch1_arr) == len(times):
					t_plot = times
				else:
					t_plot = times[-len(pitch1_arr):]
				ax_main.plot(t_plot, pitch1_arr, label='Pitch1 [deg]', color='green', linestyle='--')
			if len(pitch2_arr) > 0 and len(times) > 0:
				if len(pitch2_arr) == len(times):
					t_plot = times
				else:
					t_plot = times[-len(pitch2_arr):]
				ax_main.plot(t_plot, pitch2_arr, label='Pitch2 [deg]', color='orange', linestyle='-.')

			# plot calibrated theta
			# plot calibrated theta
			if len(theta_calib_arr) > 0 and len(times) > 0:
				if len(theta_calib_arr) == len(times):
					t_plot = times
				else:
					# align tail
					t_plot = times[-len(theta_calib_arr):]
				ax_main.plot(t_plot, theta_calib_arr, label='Theta_calib [deg]', color='magenta')

			ax_main.set_ylim(-180, 180)
			ax_main.legend(loc='upper right')
			if len(times) > 0:
				ax_main.set_xlim(times[0], times[-1])

			# Update Figure2 (IMU poses) if available (create on demand)
			try:
				if not _headless:
					if fig2 is None:
						_create_fig2()
					if fig2 is not None and ax2_q1 is not None and ax2_q2 is not None:
						# draw IMU1 (Q1)
						try:
							ax2_q1.cla()
							ax2_q1.set_xlim(-1.1,1.1); ax2_q1.set_ylim(-1.1,1.1); ax2_q1.set_zlim(-1.1,1.1)
							if last_raw_q1 is not None:
								R1 = quat_to_dcm(last_raw_q1)
								# plot basis vectors
								for v, c in zip(R1.T, ['r','g','b']):
									ax2_q1.plot([0, v[0]], [0, v[1]], [0, v[2]], color=c)
							# draw IMU2 (Q2)
							ax2_q2.cla()
							ax2_q2.set_xlim(-1.1,1.1); ax2_q2.set_ylim(-1.1,1.1); ax2_q2.set_zlim(-1.1,1.1)
							if last_raw_q2 is not None:
								R2 = quat_to_dcm(last_raw_q2)
								for v, c in zip(R2.T, ['r','g','b']):
									ax2_q2.plot([0, v[0]], [0, v[1]], [0, v[2]], color=c)
						except Exception:
							pass
						# Update Fig3 (basis) if present and requested
						try:
							if not _headless:
								if show_basis and fig3 is None:
									_create_fig3()
								if fig3 is not None and ax3_set1 is not None and ax3_set2 is not None:
									try:
										ax3_set1.cla(); ax3_set2.cla()
										ax3_set1.set_xlim(-1.1,1.1); ax3_set1.set_ylim(-1.1,1.1); ax3_set1.set_zlim(-1.1,1.1)
										ax3_set2.set_xlim(-1.1,1.1); ax3_set2.set_ylim(-1.1,1.1); ax3_set2.set_zlim(-1.1,1.1)
										ax3_set1.set_title('Set1 basis (e1,e2,e3)')
										ax3_set2.set_title('Set2 basis (e1,e2,e3)')
										# derive bases
										if last_raw_q1 is not None:
											R1 = quat_to_dcm(last_raw_q1)
										else:
											R1 = np.eye(3)
										if last_raw_q2 is not None:
											R2 = quat_to_dcm(last_raw_q2)
										else:
											R2 = np.eye(3)
										# draw selected basis vectors
										for idx, flag in enumerate((draw_e1, draw_e2, draw_e3)):
											if flag:
												v1 = R1[:, idx]
												v2 = R2[:, idx]
												col = ['r','g','b'][idx]
												ax3_set1.plot([0, v1[0]], [0, v1[1]], [0, v1[2]], color=col)
												ax3_set2.plot([0, v2[0]], [0, v2[1]], [0, v2[2]], color=col)
										# request redraw
										try:
											fig3.canvas.draw_idle()
											plt.pause(0.001)
										except Exception:
												pass
									except Exception:
										pass
								else:
									# fig3 not present; nothing to do
									pass
							else:
								pass
						except Exception:
							pass
			except Exception:
				pass

	return []

# アニメーションの設定
ani = FuncAnimation(fig, update, init_func=init, blit=False, interval=20, cache_frame_data=False)

# レイアウト調整: 下部にウィジェット領域を確保
fig.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.22, hspace=0.4)

# Port selection: TextBox with Prev/Next cycle and Rescan
scan_ports()
port_txt_ax = fig.add_axes([0.05, 0.12, 0.12, 0.05])
port_box = TextBox(port_txt_ax, 'Port', initial=(selected_port if selected_port else ''))

prev_ax = fig.add_axes([0.18, 0.12, 0.05, 0.05])
prev_btn = Button(prev_ax, '<')
next_ax = fig.add_axes([0.24, 0.12, 0.05, 0.05])
next_btn = Button(next_ax, '>')

rescan_ax = fig.add_axes([0.30, 0.12, 0.06, 0.05])
rescan_btn = Button(rescan_ax, 'Scan')

def _update_port_text():
	global selected_index, selected_port
	if available_ports:
		selected_index = max(0, min(selected_index, len(available_ports)-1))
		selected_port = available_ports[selected_index]
		try:
			port_box.set_val(selected_port)
		except Exception:
			pass
	else:
		selected_port = None
		try:
			port_box.set_val('')
		except Exception:
			pass

def on_prev(event):
	global selected_index
	if available_ports:
		selected_index = (selected_index - 1) % len(available_ports)
		_update_port_text()

def on_next(event):
	global selected_index
	if available_ports:
		selected_index = (selected_index + 1) % len(available_ports)
		_update_port_text()

def on_rescan(event):
	scan_ports()
	_update_port_text()
	try:
		print('Scan: found ports ->', available_ports)
	except Exception:
		pass

prev_btn.on_clicked(on_prev)
next_btn.on_clicked(on_next)
rescan_btn.on_clicked(on_rescan)

# Open/Close button for serial (position adjusted)
open_ax = fig.add_axes([0.38, 0.12, 0.08, 0.05])
open_btn = Button(open_ax, 'Open')
def toggle_serial_connection(button):
	global ser, selected_port, controls_enabled
	try:
		if ser is None:
			if not selected_port:
				print('No port selected')
				return
			ser = serial.Serial(selected_port, BAUD_RATE, timeout=0.1)
			try:
				button.label.set_text('Close')
			except Exception:
				pass
			print(f'Opened serial port {selected_port}')
			# enable controls that depend on open serial
			try:
				controls_enabled = True
			except Exception:
				pass
			try:
				update_button_states()
			except Exception:
				pass
		else:
			try:
				ser.close()
			except Exception:
				pass
			ser = None
			try:
				button.label.set_text('Open')
			except Exception:
				pass
			print('Closed serial port')
			# disable controls when closed
			try:
				controls_enabled = False
			except Exception:
				pass
			try:
				update_button_states()
			except Exception:
				pass
	except Exception as e:
		print('Serial open/close error:', e)

open_btn.on_clicked(lambda event: toggle_serial_connection(open_btn))

# control enable/disable state for Get/Save buttons
controls_enabled = False

def update_button_states():
	"""Enable/disable Get0/Get90/Save buttons by changing label and appearance."""
	global controls_enabled
	# helper to safely set label
	def _set_label(btn, text):
		try:
			btn.label.set_text(text)
		except Exception:
			pass
	def _set_label_color(btn, color):
		try:
			btn.label.set_color(color)
		except Exception:
			pass
	try:
		if controls_enabled:
			_set_label(get0_btn, 'Get 0deg')
			_set_label(get90_btn, 'Get 90deg')
			_set_label(save_btn, 'Save')
			try:
				get0_btn.ax.set_facecolor('white')
				get90_btn.ax.set_facecolor('white')
				save_btn.ax.set_facecolor('white')
			except Exception:
				pass
			# restore label color
			_set_label_color(get0_btn, 'black')
			_set_label_color(get90_btn, 'black')
			_set_label_color(save_btn, 'black')
		else:
			_set_label(get0_btn, '-')
			_set_label(get90_btn, '-')
			_set_label(save_btn, '-')
			try:
				get0_btn.ax.set_facecolor('0.9')
				get90_btn.ax.set_facecolor('0.9')
				save_btn.ax.set_facecolor('0.9')
			except Exception:
				pass
			# dim label color when disabled
			_set_label_color(get0_btn, '0.4')
			_set_label_color(get90_btn, '0.4')
			_set_label_color(save_btn, '0.4')
	except Exception:
		pass

# If a command-line argument is provided, treat it as desired COM port and auto-open it
try:
	if len(sys.argv) > 1:
		req_port = sys.argv[1]
		scan_ports()
		if req_port in available_ports:
			selected_index = available_ports.index(req_port)
			selected_port = req_port
			try:
				port_box.set_val(selected_port)
			except Exception:
				pass
			try:
				ser = serial.Serial(selected_port, BAUD_RATE, timeout=0.1)
				try:
					open_btn.label.set_text('Close')
				except Exception:
					pass
            
				print(f'Auto-opened serial port {selected_port}')
				# enable controls when auto-open succeeds
				try:
					controls_enabled = True
				except Exception:
					pass
				try:
					update_button_states()
				except Exception:
					pass
			except Exception as e:
				print('Auto open failed:', e)
		else:
			print('Requested port not found during scan:', req_port)
except Exception:
	pass

# Zero calibration button
get0_ax = fig.add_axes([0.48, 0.12, 0.12, 0.05])
get0_btn = Button(get0_ax, 'Get 0deg')
get90_ax = fig.add_axes([0.62, 0.12, 0.12, 0.05])
get90_btn = Button(get90_ax, 'Get 90deg')

def _compute_and_store_dcms(which, mean_override=None):
	"""which: '0' or '90' - capture last raw Y/R/P and store DCMs for both sensors.
	Also compute theta0/theta90 as circular mean of last up to 20 raw pitch-difference samples."""
	global theta0, theta90
	try:
		# Determine mean raw Theta (Y-rotation from Q3) from override or recent samples
		if mean_override is not None:
			mean_theta_raw = mean_override
		else:
				samples = list(theta_raw_buffer)[-20:]
				if len(samples) > 0:
					mean_theta_raw = circular_mean_deg(samples)
				else:
					mean_theta_raw = 0.0

		if which == '0':
			theta0 = mean_theta_raw
			print('Captured 0deg and set theta0=', theta0)
		else:
			theta90 = mean_theta_raw
			print('Captured 90deg and set theta90=', theta90)
	except Exception as e:
		print('Failed to set theta calibration:', e)

def get0_pressed(event):
	global capture_mode, capture_buffer, controls_enabled
	# disable if controls not enabled (serial not open)
	if not controls_enabled:
		return
	# start capture of CAPTURE_SAMPLES samples; prevent re-entry
	if capture_mode is not None:
		return
	capture_mode = '0'
	capture_buffer = []
	try:
		get0_btn.label.set_text('Capturing')
	except Exception:
		pass

def get90_pressed(event):
	global capture_mode, capture_buffer, controls_enabled
	# disable if controls not enabled (serial not open)
	if not controls_enabled:
		return
	# start capture of CAPTURE_SAMPLES samples; prevent re-entry
	if capture_mode is not None:
		return
	capture_mode = '90'
	capture_buffer = []
	try:
		get90_btn.label.set_text('Capturing')
	except Exception:
		pass

get0_btn.on_clicked(get0_pressed)
get90_btn.on_clicked(get90_pressed)

# --- CSV filename textbox and choose/start/stop buttons ---
fname_ax = fig.add_axes([0.05, 0.05, 0.30, 0.05])
fname_box = TextBox(fname_ax, 'File', initial=csv_filename)

choose_ax = fig.add_axes([0.36, 0.05, 0.12, 0.05])
choose_btn = Button(choose_ax, 'Choose...')

start_ax = fig.add_axes([0.50, 0.05, 0.12, 0.05])
save_btn = Button(start_ax, 'Save')

def choose_file(event):
	global csv_filename
	try:
		import tkinter as tk
		from tkinter import filedialog
		root = tk.Tk(); root.withdraw()
		fname = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV', '*.csv')])
		root.destroy()
		if fname:
			csv_filename = fname
			fname_box.set_val(csv_filename)
	except Exception:
		# fallback: use textbox value
		csv_filename = fname_box.text

def toggle_save(event):
	global recording, csv_filename, csv_fh, csv_writer, save_btn
	global save_time
	# toggle recording state and update button label
	if not recording:
		# prevent starting save when serial not open
		global controls_enabled
		if not controls_enabled:
			print('Cannot start saving: serial port not open')
			return
		# start saving
		if not csv_filename:
			csv_filename = 'data.csv'
			fname_box.set_val(csv_filename)
		try:
			csv_fh = open(csv_filename, 'w', newline='')
			csv_writer = csv.writer(csv_fh)
			# ヘッダ: t[s], theta_calib, theta, q1_w,q1_x,q1_y,q1_z, q2_w,q2_x,q2_y,q2_z
			header = ['t[s]', 'theta_calib', 'theta', 'q1_w','q1_x','q1_y','q1_z', 'q2_w','q2_x','q2_y','q2_z']
			csv_writer.writerow(header)
			csv_fh.flush()
			# reset save_time to zero at start
			save_time = 0.0
			recording = True
			try:
				save_btn.label.set_text('Stop')
				fig.canvas.draw_idle()
			except Exception:
				pass
		except Exception as e:
			print('Failed to open CSV:', e)
	else:
		# stop saving
		recording = False
		try:
			if csv_fh is not None:
				csv_fh.close()
		except Exception:
			pass
		csv_fh = None
		csv_writer = None
		try:
			save_btn.label.set_text('Save')
			fig.canvas.draw_idle()
		except Exception:
			pass

choose_btn.on_clicked(choose_file)
save_btn.on_clicked(toggle_save)

# initialize button states according to serial connection
try:
	update_button_states()
except Exception:
	pass

# Show Basis checkbox in main figure: when checked, show fig3 window
show_ax = fig.add_axes([0.72, 0.12, 0.08, 0.05])
show_cb = CheckButtons(show_ax, ['Show Basis'], [show_basis])
def _on_show_basis(label):
	global show_basis, fig3, ax3_set1, ax3_set2, fig2, ax2_q1, ax2_q2
	show_basis = not show_basis
	# create or destroy fig3 and fig2 windows on demand
	try:
		if show_basis:
			if fig3 is None:
				_create_fig3()
			if fig3 is not None:
				try:
					fig3.canvas.draw_idle()
				except Exception:
					pass
			# debug
			try:
				print('ShowBasis: requested show (headless=', _headless, ')')
			except Exception:
				pass
			# also create Figure2 for IMU poses
			if fig2 is None:
				_create_fig2()
			if fig2 is not None:
				try:
					fig2.canvas.draw_idle()
					try:
						fig2.show()
					except Exception:
						pass
					try:
						plt.pause(0.001)
					except Exception:
						pass
				except Exception:
					pass
			# ensure fig3 also shown
			try:
				if fig3 is not None:
					try:
						fig3.show()
					except Exception:
						pass
					try:
						plt.pause(0.001)
					except Exception:
						pass
			except Exception:
				pass
		else:
			# close and clear fig3 if it exists
			try:
				if fig3 is not None:
					plt.close(fig3)
			except Exception:
				pass
			# clear fig3 references
			try:
				fig3 = None
				ax3_set1 = None
				ax3_set2 = None
			except Exception:
				pass
			# close and clear fig2 if it exists
			try:
				if fig2 is not None:
					plt.close(fig2)
			except Exception:
				pass
			try:
				fig2 = None
				ax2_q1 = None
				ax2_q2 = None
			except Exception:
				pass
	except Exception:
		pass

show_cb.on_clicked(_on_show_basis)
# fig3 is created on demand; if it's already present and should be hidden, withdraw it
if not show_basis and fig3 is not None:
	try:
		mgr = fig3.canvas.manager
		win = getattr(mgr, 'window', None)
		if win is not None:
			try:
				win.withdraw()
			except Exception:
				try:
					win.wm_withdraw()
				except Exception:
					pass
	except Exception:
		pass

# Quit ボタン
quit_ax = fig.add_axes([0.78, 0.05, 0.12, 0.05])
quit_btn = Button(quit_ax, 'Quit')

def quit_pressed(event):
	global ser, csv_fh, controls_enabled
	try:
		if 'ani' in globals():
			try:
				ani.event_source.stop()
			except Exception:
				pass
	except Exception:
		pass
	try:
		ser.close()
	except Exception:
		pass
	try:
		if csv_fh is not None:
			csv_fh.close()
	except Exception:
		pass
	# ensure controls show disabled state
	try:
		controls_enabled = False
	except Exception:
		pass
	try:
		update_button_states()
	except Exception:
		pass
	plt.close('all')

quit_btn.on_clicked(quit_pressed)

# GUI が利用できる環境では表示、それ以外（ヘッドレス）では保存する
if not _headless:
	plt.show()
else:
	# アニメーションや ffmpeg の有無により保存に失敗することがあるため例外処理でフォールバック
	try:
		if 'ani' in globals():
			try:
				ani.save('animation.mp4', writer='ffmpeg', fps=30)
				print('Saved animation to animation.mp4')
			except Exception:
				# ffmpeg がない等で保存できない場合は静止画を保存
				try:
					fig.savefig('figure.png')
					print('Headless: saved figure.png')
				except Exception:
					print('Headless: could not save animation or figure')
	except Exception:
		pass

try:
	if ser is not None:
		ser.close()
except Exception:
	pass
try:
	if csv_fh is not None:
		csv_fh.close()
except Exception:
	pass
