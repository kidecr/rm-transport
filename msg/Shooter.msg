# 射击子弹数量 0为不射击，n为射击n发(n>0)，-1为持续射击（此时持续发射，给0才会停）
int32 bulletnum
# 1为高射速模式，2为低射速模式（从电控获取状态）
bool shoot_mode
# 获取连发剩余子弹数
int32 getbulletnumber
# 开弹仓 1开 0关
bool box
# 停止发射，包括摩擦轮 , 1停，0不处理
bool stop
# 时间戳
float64 shoot_timestamp