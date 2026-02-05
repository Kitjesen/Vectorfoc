import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def analyze():
    filename = 'sim_response.csv'
    if not os.path.exists(filename):
        print(f"Error: {filename} not found.")
        return

    df = pd.read_csv(filename)
    
    # Calc Metrics
    target = df['RefVel'].iloc[-1]
    final_vel = df['ActVel'].iloc[-1]
    error = abs(target - final_vel)
    overshoot = (df['ActVel'].max() - target) / target * 100 if target != 0 else 0
    
    print(f"Analysis Results:")
    print(f"Target Vel: {target} rad/s")
    print(f"Final Vel:  {final_vel:.2f} rad/s")
    print(f"Error:      {error:.4f} rad/s")
    print(f"Overshoot:  {overshoot:.2f}%")
    
    # Plot
    fig, ax = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    # 1. Velocity
    ax[0].plot(df['Time'], df['RefVel'], 'r--', label='Target')
    ax[0].plot(df['Time'], df['ActVel'], 'b-', label='Actual')
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
    
    # 3. Currents AlphaBeta (Plant)
    ax[2].plot(df['Time'], df['Ialpha_Sim'], 'c-', label='Ialpha')
    ax[2].plot(df['Time'], df['Ibeta_Sim'], 'y-', label='Ibeta')
    ax[2].set_ylabel('Phase Current [A]')
    ax[2].set_xlabel('Time [s]')
    ax[2].legend()
    ax[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('foc_response.png')
    print("Plot saved to foc_response.png")

if __name__ == "__main__":
    analyze()
