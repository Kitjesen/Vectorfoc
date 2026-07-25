import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def analyze():
    filename = sys.argv[1] if len(sys.argv) > 1 else 'sim_response.csv'
    if not os.path.exists(filename):
        print(f"Error: {filename} not found.")
        return

    df = pd.read_csv(filename)

    ref_col = 'RefVel' if 'RefVel' in df.columns else 'RefSpeed'
    actual_col = 'ActVel' if 'ActVel' in df.columns else 'ActualSpeed'
    if ref_col not in df.columns or actual_col not in df.columns:
        print(f"Error: {filename} does not contain velocity response columns.")
        return

    target = df[ref_col].iloc[-1]
    final_vel = df[actual_col].iloc[-1]
    error = abs(target - final_vel)
    last_switch_index = df.index[df[ref_col] != df[ref_col].shift()].max()
    if pd.isna(last_switch_index):
        post_switch = df
    else:
        post_switch = df.loc[last_switch_index:]
    overshoot = post_switch[actual_col].max() - target

    print(f"Analysis Results:")
    print(f"Target Vel: {target} rad/s")
    print(f"Final Vel:  {final_vel:.2f} rad/s")
    print(f"Error:      {error:.4f} rad/s")
    print(f"Overshoot:  {overshoot:.2f} rad/s")

    # Plot
    fig, ax = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # 1. Velocity
    ax[0].plot(df['Time'], df[ref_col], 'r--', label='Target')
    ax[0].plot(df['Time'], df[actual_col], 'b-', label='Actual')
    ax[0].set_ylabel('Velocity [rad/s]')
    ax[0].set_title('Step Response')
    ax[0].legend()
    ax[0].grid(True)

    # 2. Currents DQ
    ax[1].plot(df['Time'], df['Iq'], 'g-', label='Iq')
    ax[1].plot(df['Time'], df['Id'], 'm-', label='Id')
    ax[1].set_ylabel('Current [A]')
    ax[1].set_title('DQ Currents')
    ax[1].legend()
    ax[1].grid(True)

    # 3. Plant currents for legacy logs; tracking error for setpoint-switch logs.
    if 'Ialpha_Sim' in df.columns and 'Ibeta_Sim' in df.columns:
        ax[2].plot(df['Time'], df['Ialpha_Sim'], 'c-', label='Ialpha')
        ax[2].plot(df['Time'], df['Ibeta_Sim'], 'y-', label='Ibeta')
        ax[2].set_ylabel('Phase Current [A]')
    else:
        ax[2].plot(df['Time'], df[actual_col] - df[ref_col], 'k-', label='Actual - Target')
        ax[2].axhline(0.0, color='0.5', linestyle='--', linewidth=1)
        ax[2].set_ylabel('Velocity Error [rad/s]')
    ax[2].set_xlabel('Time [s]')
    ax[2].legend()
    ax[2].grid(True)

    plt.tight_layout()
    output = os.path.splitext(os.path.basename(filename))[0] + '.png'
    plt.savefig(output)
    print(f"Plot saved to {output}")

if __name__ == "__main__":
    analyze()
