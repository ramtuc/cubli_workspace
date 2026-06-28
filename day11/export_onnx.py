import torch
import torch.nn as nn
from stable_baselines3 import PPO

# 1. 学習済みモデルのロード
# device="cpu" を指定し、CPU 推論前提のモデルとしてロード
model_path = "models/PPO_Robust/final_model.zip"
model = PPO.load(model_path, device="cpu")

# 2. STM32 実装用ラッパーモデルの定義
# 推論に必要な Actor ネットワークのみを抽出して実行するクラス
class OnnxablePolicy(nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, observation):
        # SB3 の内部メソッドを直接呼ぶことで Tensor のまま処理を流す
        features = self.policy.extract_features(observation)
        latent_pi, _ = self.policy.mlp_extractor(features)
        action = self.policy.action_net(latent_pi)
        return action

onnx_policy = OnnxablePolicy(model.policy)

# 3. ダミー入力の生成（入力次元数: 6）
# [Roll, Pitch, GyroX, GyroY, GyroZ, WheelSpeed]
dummy_input = torch.randn(1, 6)

# 4. ONNX エクスポート実行
torch.onnx.export(
    onnx_policy,
    dummy_input,
    "cubli_policy.onnx",    # 出力ファイル名
    opset_version=11,       # X-CUBE-AI 推奨バージョン
    input_names=["input"],
    output_names=["output"]
)

print("Conversion Complete: cubli_policy.onnx generated.")