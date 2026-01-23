
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path

def _normalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize dataframe column names.

    Some CSVs in this repo have leading spaces after commas, e.g. ['err_x', ' err_y', ' err_z'].
    This helper strips whitespace so downstream code can reliably use canonical names.
    """

    df = df.copy()
    df.columns = [str(c).strip() for c in df.columns]
    return df


def _plot_if_columns_exist(ax, df: pd.DataFrame, columns, labels, *, transform=None, title=None, xlabel=None, ylabel=None):
    """Plot selected columns if they exist; warn (print) otherwise."""

    missing = [c for c in columns if c not in df.columns]
    if missing:
        print(f"[WARN] {title or 'plot'}: missing columns {missing}. Available columns: {list(df.columns)}")

    for c, lab in zip(columns, labels):
        if c not in df.columns:
            continue
        y = df[c]
        if transform is not None:
            y = transform(y)
        ax.plot(y, label=lab, alpha=0.8)

    if title:
        ax.set_title(title, fontsize=12)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.legend()
    ax.grid(True)

# 设置中文显示和图形样式
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.style.use('seaborn-v0_8-whitegrid')

# 数据目录
data_dir = Path('/home/lee/Code/quadruped_control_ws/src/quadruped_ros2_control/logs/data')

# 批量加载数据
data = {}
for csv_file in data_dir.glob('*.csv'):
    try:
        data[csv_file.stem] = _normalize_columns(pd.read_csv(csv_file))
        # print(f"Loaded: {csv_file.stem} - Shape: {data[csv_file.stem].shape}")
    except Exception as e:
        print(f"Error loading {csv_file.stem}: {e}")

print(f"\n共加载 {len(data)} 个数据文件")
fig, axes = plt.subplots(2, 2, figsize=(16, 12))

print("===========================估计值与gazebo真实值误差分析===========================")

# 位置误差
ax = axes[0, 0]
pos_err = data['posEstiTruthError']
# _plot_if_columns_exist(
#     ax,
#     pos_err,
#     columns=['err_x', 'err_y', 'err_z'],
#     labels=['X', 'Y', 'Z'],
#     title='Position Estimation Error',
#     xlabel='Time Step',
#     ylabel='Error (m)',
# )

# 速度误差
ax = axes[0, 1]
vel_err = data['velEstiTruthError']
# _plot_if_columns_exist(
#     ax,
#     vel_err,
#     columns=['err_vx', 'err_vy', 'err_vz'],
#     labels=['Vx', 'Vy', 'Vz'],
#     title='Velocity Estimation Error',
#     xlabel='Time Step',
#     ylabel='Error (m/s)',
# )

# 姿态误差
ax = axes[1, 0]
rot_err = data['rotEstiTruthError']
# _plot_if_columns_exist(
#     ax,
#     rot_err,
#     columns=['err_yaw', 'err_pitch', 'err_roll'],
#     labels=['Yaw', 'Pitch', 'Roll'],
#     transform=np.rad2deg,
#     title='Rotation Estimation Error',
#     xlabel='Time Step',
#     ylabel='Error (deg)',
# )

# 角速度误差
ax = axes[1, 1]
angvel_err = data['angvelEstiTruthError']
# _plot_if_columns_exist(
#     ax,
#     angvel_err,
#     columns=['err_wx', 'err_wy', 'err_wz'],
#     labels=['Wx', 'Wy', 'Wz'],
#     transform=np.rad2deg,
#     title='Angular Velocity Estimation Error',
#     xlabel='Time Step',
#     ylabel='Error (deg/s)',
# )

# plt.tight_layout()
# plt.show()

# 打印误差统计
print("=" * 60)
print("误差统计分析")
print("=" * 60)

print("\n【位置误差统计 (m)】")
for col in pos_err.columns:
    print(f"  {col}: 均值={pos_err[col].mean():.6f}, 标准差={pos_err[col].std():.6f}, 最大值={pos_err[col].abs().max():.6f}")

print("\n【速度误差统计 (m/s)】")
for col in vel_err.columns:
    print(f"  {col}: 均值={vel_err[col].mean():.6f}, 标准差={vel_err[col].std():.6f}, 最大值={vel_err[col].abs().max():.6f}")

print("\n【姿态误差统计 (deg)】")
for col in rot_err.columns:
    vals = np.rad2deg(rot_err[col])
    print(f"  {col}: 均值={vals.mean():.4f}, 标准差={vals.std():.4f}, 最大值={vals.abs().max():.4f}")

print("\n【角速度误差统计 (deg/s)】")
for col in angvel_err.columns:
    vals = np.rad2deg(angvel_err[col])
    print(f"  {col}: 均值={vals.mean():.4f}, 标准差={vals.std():.4f}, 最大值={vals.abs().max():.4f}")


print("======================================================")
print("=====================协方差分析=======================")
fig, axes = plt.subplots(2, 3, figsize=(18, 10))

# 位置协方差
ax = axes[0, 0]
p_pos = data['P_diag_position']
ax.plot(np.sqrt(p_pos['P_pos_x']), label='σ_x', alpha=0.8)
ax.plot(np.sqrt(p_pos['P_pos_y']), label='σ_y', alpha=0.8)
ax.plot(np.sqrt(p_pos['P_pos_z']), label='σ_z', alpha=0.8)
ax.set_title('Position Covariance (σ)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Standard Deviation (m)')
ax.legend()
ax.grid(True)

# 速度协方差
ax = axes[0, 1]
p_vel = data['P_diag_velocity']
ax.plot(np.sqrt(p_vel['P_vel_x']), label='σ_vx', alpha=0.8)
ax.plot(np.sqrt(p_vel['P_vel_y']), label='σ_vy', alpha=0.8)
ax.plot(np.sqrt(p_vel['P_vel_z']), label='σ_vz', alpha=0.8)
ax.set_title('Velocity Covariance (σ)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Standard Deviation (m/s)')
ax.legend()
ax.grid(True)

# 姿态协方差
ax = axes[0, 2]
p_rot = data['P_diag_rotation']
ax.plot(np.rad2deg(np.sqrt(p_rot['P_rot_x'])), label='σ_roll', alpha=0.8)
ax.plot(np.rad2deg(np.sqrt(p_rot['P_rot_y'])), label='σ_pitch', alpha=0.8)
ax.plot(np.rad2deg(np.sqrt(p_rot['P_rot_z'])), label='σ_yaw', alpha=0.8)
ax.set_title('Rotation Covariance (σ)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Standard Deviation (deg)')
ax.legend()
ax.grid(True)

# 加速度计偏置协方差
ax = axes[1, 0]
p_ba = data['P_diag_acc_ba']
ax.plot(np.sqrt(p_ba.iloc[:, 0]), label='σ_ba_x', alpha=0.8)
ax.plot(np.sqrt(p_ba.iloc[:, 1]), label='σ_ba_y', alpha=0.8)
ax.plot(np.sqrt(p_ba.iloc[:, 2]), label='σ_ba_z', alpha=0.8)
ax.set_title('Accelerometer Bias Covariance (σ)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Standard Deviation (m/s²)')
ax.legend()
ax.grid(True)

# 陀螺仪偏置协方差
ax = axes[1, 1]
p_bg = data['P_diag_gyro_bg']
ax.plot(np.rad2deg(np.sqrt(p_bg.iloc[:, 0])), label='σ_bg_x', alpha=0.8)
ax.plot(np.rad2deg(np.sqrt(p_bg.iloc[:, 1])), label='σ_bg_y', alpha=0.8)
ax.plot(np.rad2deg(np.sqrt(p_bg.iloc[:, 2])), label='σ_bg_z', alpha=0.8)
ax.set_title('Gyroscope Bias Covariance (σ)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Standard Deviation (deg/s)')
ax.legend()
ax.grid(True)

# 足端位置协方差 (以足端1为例)
ax = axes[1, 2]
p_foot1 = data['P_diag_foot1_pos']
ax.plot(np.sqrt(p_foot1.iloc[:, 0]), label='σ_foot1_x', alpha=0.8)
ax.plot(np.sqrt(p_foot1.iloc[:, 1]), label='σ_foot1_y', alpha=0.8)
ax.plot(np.sqrt(p_foot1.iloc[:, 2]), label='σ_foot1_z', alpha=0.8)
ax.set_title('Foot 1 Position Covariance (σ)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Standard Deviation (m)')
ax.legend()
ax.grid(True)

# plt.tight_layout()
# plt.show()

print("=" * 60)
print("协方差收敛性分析")
print("=" * 60)
print(f"\n位置协方差最终值: σ_x={np.sqrt(p_pos['P_pos_x'].iloc[-1]):.6f}m, σ_y={np.sqrt(p_pos['P_pos_y'].iloc[-1]):.6f}m, σ_z={np.sqrt(p_pos['P_pos_z'].iloc[-1]):.6f}m")
print(f"速度协方差最终值: σ_vx={np.sqrt(p_vel['P_vel_x'].iloc[-1]):.6f}m/s, σ_vy={np.sqrt(p_vel['P_vel_y'].iloc[-1]):.6f}m/s, σ_vz={np.sqrt(p_vel['P_vel_z'].iloc[-1]):.6f}m/s")

print("======================================================")
print("=====================观测残差分析=======================")


fig, axes = plt.subplots(3, 2, figsize=(16, 14))

# 足端位置残差
for i in range(4):
    key = f'y_pos_residual_foot_{i}'
    if key in data:
        residual = data[key]
        ax = axes[0, 0] if i < 2 else axes[0, 1]
        # 过滤掉零值（摆动相）
        for col in residual.columns:
            vals = residual[col].replace(0, np.nan)
            ax.plot(vals, alpha=0.6, label=f'Foot{i}_{col.split("_")[-1]}')
        
axes[0, 0].set_title('Foot Position Residual (Foot 0,1)', fontsize=12)
axes[0, 0].set_xlabel('Time Step')
axes[0, 0].set_ylabel('Residual (m)')
axes[0, 0].legend(loc='upper right', fontsize=8)
axes[0, 0].grid(True)

axes[0, 1].set_title('Foot Position Residual (Foot 2,3)', fontsize=12)
axes[0, 1].set_xlabel('Time Step')
axes[0, 1].set_ylabel('Residual (m)')
axes[0, 1].legend(loc='upper right', fontsize=8)
axes[0, 1].grid(True)

# 足端速度残差
for i in range(4):
    key = f'y_vel_residual_foot_{i}'
    if key in data:
        residual = data[key]
        ax = axes[1, 0] if i < 2 else axes[1, 1]
        for col in residual.columns:
            vals = residual[col].replace(0, np.nan)
            ax.plot(vals, alpha=0.6, label=f'Foot{i}_{col.split("_")[-1]}')

axes[1, 0].set_title('Foot Velocity Residual (Foot 0,1)', fontsize=12)
axes[1, 0].set_xlabel('Time Step')
axes[1, 0].set_ylabel('Residual (m/s)')
axes[1, 0].legend(loc='upper right', fontsize=8)
axes[1, 0].grid(True)

axes[1, 1].set_title('Foot Velocity Residual (Foot 2,3)', fontsize=12)
axes[1, 1].set_xlabel('Time Step')
axes[1, 1].set_ylabel('Residual (m/s)')
axes[1, 1].legend(loc='upper right', fontsize=8)
axes[1, 1].grid(True)

# 高度残差
ax = axes[2, 0]
if 'y_height02_residual' in data:
    h02 = data['y_height02_residual']
    for col in h02.columns:
        vals = h02[col].replace(0, np.nan)
        ax.plot(vals, alpha=0.7, label=col)
ax.set_title('Height Residual (Foot 0,2)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Residual (m)')
ax.legend()
ax.grid(True)

ax = axes[2, 1]
if 'y_height13_residual' in data:
    h13 = data['y_height13_residual']
    for col in h13.columns:
        vals = h13[col].replace(0, np.nan)
        ax.plot(vals, alpha=0.7, label=col)
ax.set_title('Height Residual (Foot 1,3)', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Residual (m)')
ax.legend()
ax.grid(True)

# plt.tight_layout()
# plt.show()

# 残差统计
print("=" * 60)
print("观测残差统计分析")
print("=" * 60)
for i in range(4):
    pos_key = f'y_pos_residual_foot_{i}'
    vel_key = f'y_vel_residual_foot_{i}'
    if pos_key in data:
        pos_res = data[pos_key]
        # 只统计非零值
        for col in pos_res.columns:
            non_zero = pos_res[col][pos_res[col] != 0]
            if len(non_zero) > 0:
                print(f"位置残差 Foot{i} {col}: 均值={non_zero.mean():.6f}, 标准差={non_zero.std():.6f}")


                fig, axes = plt.subplots(2, 2, figsize=(14, 10))


print("======================================================")
print("=====================偏置分析=======================")

# 加速度计偏置估计
ax = axes[0, 0]
ba = data['state_ba']
ax.plot(ba['ba_x'], label='ba_x', alpha=0.8)
ax.plot(ba['ba_y'], label='ba_y', alpha=0.8)
ax.plot(ba['ba_z'], label='ba_z', alpha=0.8)
ax.set_title('Accelerometer Bias Estimation', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Bias (m/s²)')
ax.legend()
ax.grid(True)

# 陀螺仪偏置估计
ax = axes[0, 1]
bg = data['state_bg']
ax.plot(np.rad2deg(bg['bg_x']), label='bg_x', alpha=0.8)
ax.plot(np.rad2deg(bg['bg_y']), label='bg_y', alpha=0.8)
ax.plot(np.rad2deg(bg['bg_z']), label='bg_z', alpha=0.8)
ax.set_title('Gyroscope Bias Estimation', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Bias (deg/s)')
ax.legend()
ax.grid(True)

# IMU原始加速度数据
ax = axes[1, 0]
if 'IMU_acc_x' in data and 'IMU_acc_y' in data and 'IMU_acc_z' in data:
    ax.plot(data['IMU_acc_x'].iloc[:, 0], label='acc_x', alpha=0.6)
    ax.plot(data['IMU_acc_y'].iloc[:, 0], label='acc_y', alpha=0.6)
    ax.plot(data['IMU_acc_z'].iloc[:, 0], label='acc_z', alpha=0.6)
ax.set_title('Raw IMU Acceleration', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Acceleration (m/s²)')
ax.legend()
ax.grid(True)

# IMU原始陀螺仪数据
ax = axes[1, 1]
if 'IMU_gyro_x' in data and 'IMU_gyro_y' in data and 'IMU_gyro_z' in data:
    ax.plot(np.rad2deg(data['IMU_gyro_x'].iloc[:, 0]), label='gyro_x', alpha=0.6)
    ax.plot(np.rad2deg(data['IMU_gyro_y'].iloc[:, 0]), label='gyro_y', alpha=0.6)
    ax.plot(np.rad2deg(data['IMU_gyro_z'].iloc[:, 0]), label='gyro_z', alpha=0.6)
ax.set_title('Raw IMU Gyroscope', fontsize=12)
ax.set_xlabel('Time Step')
ax.set_ylabel('Angular Velocity (deg/s)')
ax.legend()
ax.grid(True)

# plt.tight_layout()
# plt.show()

print("=" * 60)
print("IMU偏置估计最终值")
print("=" * 60)
print(f"加速度计偏置: ba_x={ba['ba_x'].iloc[-1]:.6f}, ba_y={ba['ba_y'].iloc[-1]:.6f}, ba_z={ba['ba_z'].iloc[-1]:.6f} m/s²")
print(f"陀螺仪偏置: bg_x={np.rad2deg(bg['bg_x'].iloc[-1]):.4f}, bg_y={np.rad2deg(bg['bg_y'].iloc[-1]):.4f}, bg_z={np.rad2deg(bg['bg_z'].iloc[-1]):.4f} deg/s")


print("======================================================")
print("=====================估计值与gazebo真实值分析=======================")


fig, axes = plt.subplots(3, 3, figsize=(18, 14))

state_pos = data['state_pos']
true_pos = data['gazebo_true_position']
state_vel = data['state_vel']
true_vel = data['gazebo_true_velocity']
state_rot = data['state_rot_zyx']
true_rot = data['gazebo_true_rotation']

# 位置对比
for i, (est_col, true_col, axis_name) in enumerate([
    ('pos_x', 'gt_x', 'X'),
    ('pos_y', 'gt_y', 'Y'),
    ('pos_z', 'gt_z', 'Z')
]):
    ax = axes[0, i]
    if est_col not in state_pos.columns or true_col not in true_pos.columns:
        print(f"[WARN] Position {axis_name}: missing columns. est={est_col in state_pos.columns}, gt={true_col in true_pos.columns}. est_cols={list(state_pos.columns)}, gt_cols={list(true_pos.columns)}")
        ax.set_title(f'Position {axis_name}', fontsize=12)
        ax.grid(True)
        continue

    ax.plot(state_pos[est_col], label='Estimated', alpha=0.8)
    ax.plot(true_pos[true_col], label='Ground Truth', alpha=0.8, linestyle='--')
    ax.set_title(f'Position {axis_name}', fontsize=12)
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Position (m)')
    ax.legend()
    ax.grid(True)

# 速度对比
for i, (est_col, true_col, axis_name) in enumerate([
    ('vel_x', 'gt_vx', 'X'),
    ('vel_y', 'gt_vy', 'Y'),
    ('vel_z', 'gt_vz', 'Z')
]):
    ax = axes[1, i]
    if est_col not in state_vel.columns or true_col not in true_vel.columns:
        print(f"[WARN] Velocity {axis_name}: missing columns. est={est_col in state_vel.columns}, gt={true_col in true_vel.columns}. est_cols={list(state_vel.columns)}, gt_cols={list(true_vel.columns)}")
        ax.set_title(f'Velocity {axis_name}', fontsize=12)
        ax.grid(True)
        continue

    ax.plot(state_vel[est_col], label='Estimated', alpha=0.8)
    ax.plot(true_vel[true_col], label='Ground Truth', alpha=0.8, linestyle='--')
    ax.set_title(f'Velocity {axis_name}', fontsize=12)
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Velocity (m/s)')
    ax.legend()
    ax.grid(True)

# 姿态对比
for i, (est_col, true_col, axis_name) in enumerate([
    ('rot_z', 'gt_yaw', 'Yaw'),
    ('rot_y', 'gt_pitch', 'Pitch'),
    ('rot_x', 'gt_roll', 'Roll')
]):
    ax = axes[2, i]
    if est_col not in state_rot.columns or true_col not in true_rot.columns:
        print(f"[WARN] Rotation {axis_name}: missing columns. est={est_col in state_rot.columns}, gt={true_col in true_rot.columns}. est_cols={list(state_rot.columns)}, gt_cols={list(true_rot.columns)}")
        ax.set_title(f'Rotation {axis_name}', fontsize=12)
        ax.grid(True)
        continue

    ax.plot(np.rad2deg(state_rot[est_col]), label='Estimated', alpha=0.8)
    ax.plot(np.rad2deg(true_rot[true_col]), label='Ground Truth', alpha=0.8, linestyle='--')
    ax.set_title(f'Rotation {axis_name}', fontsize=12)
    ax.set_xlabel('Time Step')
    ax.set_ylabel('Angle (deg)')
    ax.legend()
    ax.grid(True)

# plt.tight_layout()
# plt.show()