# USB_Composite
## 项目描述
本次开发是基于STM32的HAL库修改而来的，因为原HAL USB库的组合设备配置难度大，不适合直接开发。所以采用了在原库文件的基础上进行修改，极大方便了USB组合设备的开发。
## :package: 前期准备
下载STM32CubeMX软件用于配置各个外设，手动配置也可以。配置的参数如下，保持默认即可。

![STM32CubeMX配置](https://github.com/YGDL/USB_Composite/raw/main/Photo/STM32CubeMX.png "STM32CubeMX对于USB组合类配置")

主要配置用于实现存储的QSPI外设、通用串行总线的USB外设、文件管理系统FATFS以及实时操作系统FreeRTOS。这四个部分将在单片机上实现一个USB的组合设备。

完成外设以及时钟的配置后，生成代码。本人选用的集成开发环境是Keil V5.37。