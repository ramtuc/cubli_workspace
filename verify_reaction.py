from src.simulation import CubliSimulation
import pybullet as p
import time

def main():
    # クラスのインスタンス化
    sim = CubliSimulation(use_gui=True)
    
    print("========================================")
    print(" Simulation Start")
    print(" [→] 右矢印キー : 時計回りのトルク")
    print(" [←] 左矢印キー : 反時計回りのトルク")
    print(" [Q] キー       : 終了")
    print("========================================")
    
    while True:
        # キーボードの入力を取得
        keys = p.getKeyboardEvents()
        
        # トルクの初期値は0
        torque = 0.0
        
        # [→] 右矢印キーが押されているか？
        if p.B3G_RIGHT_ARROW in keys and (keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN):
            torque = 0.885    # プラスのトルク
            
        # [←] 左矢印キーが押されているか？
        elif p.B3G_LEFT_ARROW in keys and (keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN):
            torque = -0.885 # マイナスのトルク

        # [Q] キーで終了
        if ord('q') in keys and (keys[ord('q')] & p.KEY_IS_DOWN):
            break

        # シミュレーションに適用
        sim.apply_torque(torque)
        sim.step()
        
        # ログ表示（トルクがかかっている時だけ表示）
        if torque != 0:
            angle, velocity = sim.get_state()
            print(f"Torque: {torque:5.2f} | BodyAngle: {angle:6.3f} | WheelVel: {velocity:6.2f}")

    print("Finished.")

if __name__ == "__main__":
    main()