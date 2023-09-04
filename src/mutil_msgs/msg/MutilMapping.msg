Header header
uint8 usVehicleID # 无人车ID
uint16 usGridRowNum # 地图行数
uint16 usGridColumnNum # 地图列数
uint16 cGridRowScale # 地图行精度
uint16 cGridColumnScale # 地图列精度
uint16 usMsgVehicleRow # 无人车所在行
uint16 usMsgVehicleCol # 无人车所在列
uint32 uiMsgVehicleFixedX # 无人车所在经度
uint32 uiMsgVehicleFixedY # 无人车所在纬度

uint16 uiMsgVehicleFixedRoll # 无人车滚转角
uint16 uiMsgVehicleFixedPitch # 无人车俯仰角
uint16 uiMsgVehicleFixedHeading # 无人车航向角--->朝向

nav_msgs/OccupancyGrid msgGrid # 栅格地图

uint8[] ucOccupyAttibute # 栅格占据属性
uint16 usCheckSum







