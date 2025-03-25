# Thesis Outline

## 1. Introduction
- 動機：四足機器狗控制的困難（高維、延遲、不穩）
- 問題定義：在模擬中訓練可靠的步態策略，並能轉移至實體
- 貢獻摘要：
  - PPO 實作與實驗設計
  - 將策略轉換至 SAC 並比較效果
  - 探討 Domain Randomization 成效與策略穩定性
  - 模擬到實體之間的微調方法探索

## 2. Related Work
- PPO / SAC 在機器人控制上的應用
- Domain Randomization 技術
- Sim-to-Real transfer 的關鍵挑戰與方法
- 四足機器人相關控制方法

## 3. Methodology
- 模型與環境設定（PyBullet 模型、地形、初始姿態）
- PPO/SAC 訓練設定（reward、state space、action space）
- Domain Randomization 設計（哪些參數、分布範圍）
- 微調策略（如 fine-tuning、Residual Policy Learning）

## 4. Experiments
- 訓練步態與收斂結果分析
- PPO vs SAC 比較（reward 收斂、穩定性）
- 有無 Domain Randomization 的影響
- 測試 Transfer 成功率與失敗原因
- 消融實驗（不加某些隨機化或 reward 設定）

## 5. Discussion
- 學到什麼？哪些部分效果好/壞？為什麼？
- 討論 sim-to-real 的 gap 有哪些可以改善的方向

## 6. Conclusion & Future Work
- 總結貢獻與結果
- 下一步（硬體實測、自適應策略、更強的演算法）

---

## 附錄
- 模型細節、超參數表、訓練時間、硬體配置
