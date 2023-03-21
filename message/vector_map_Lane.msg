# jct
uint8 NORMAL=0
uint8 LEFT_BRANCHING=1
uint8 RIGHT_BRANCHING=2
uint8 LEFT_MERGING=3
uint8 RIGHT_MERGING=4
uint8 COMPOSITION=5

# lanetype
uint8 STRAIGHT=0
uint8 LEFT_TURN=1
uint8 RIGHT_TURN=2

# lanecfgfg
uint8 PASS=0
uint8 FAIL=1

# Ver 1.00
int32 lnid    # 识别编号
int32 did
int32 blid
int32 flid
int32 bnid    # 起始 Node 的 nid(识别编号)
int32 fnid    # 终点 Node 的 nid

# jct 指定当前车道分支/合并模式(0：正常，1：分支到左边
# 2：分支到右边，3：合并到左车道，4：合并到右车道，5：未知) 
int32 jct

# blid2-4：指定要合并到当前车道的 Lane 的 lnid(0 表示没有合并)
int32 blid2
int32 blid3
int32 blid4

# flid2-4：指定要分流出去的车道的 Lane 的 lnid
int32 flid2
int32 flid3
int32 flid4
int32 clossid   # 只使用在十字路口，输入 clossid，否则值为 0
float64 span
int32 lcnt    # 车道数
int32 lno     # 本车道编号，从左边的车道开始编号

# Ver 1.20
int32 lanetype    # 车道类型(0：直行，1：左转，2：右转) 
int32 limitvel    # 设定的最大速度
int32 refvel      # 目标运动速度(驾驶期间估计速度) 

# roadsecid 指定路段 (在十字路口是 0，相邻车道通常具有相同的 roadsecid)
int32 roadsecid
int32 lanecfgfg   # 是否车道允许变道(0：允许，1：不允许) 

# linkwaid 指定 WayArea 的标识编号, WayArea 定义了车辆可以行驶的道路区域
int32 linkwaid
