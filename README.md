# USB_Composite
## 项目描述
本次开发是基于STM32的HAL库修改而来的，因为原HAL USB库的组合设备配置难度大，不适合直接开发。所以采用了在原库文件的基础上进行修改，极大方便了USB组合设备的开发。
## 前期准备
下载STM32CubeMX软件用于配置各个外设，手动配置也可以。配置的参数如下，保持默认即可。

![STM32CubeMX配置](https://github.com/YGDL/USB_Composite/raw/main/Photo/STM32CubeMX.png "STM32CubeMX对于USB组合类配置")

主要配置用于实现存储的QSPI外设、通用串行总线的USB外设、文件管理系统FATFS以及实时操作系统FreeRTOS。这四个部分将在单片机:basketball_man:上实现一个USB的组合设备。

完成外设以及时钟的配置后，生成代码。本人选用的集成开发环境是Keil V5.37。代码生成后如下图。

![Keil视图](https://github.com/YGDL/USB_Composite/raw/main/Photo/Keil_New.png "Keil视图")

## 重写目标

打开左侧的项目树，在最下方的两个组里面包含有USB类的核心文件，如下图所示。

![USB类](https://github.com/YGDL/USB_Composite/raw/main/Photo/USB_Class_Group.png "USB类")

前面配置的时候选择了USB MSC类，因此在项目中包含了MSC类文件。其中usbd_storage_if.c属于MSC类的接口文件，与底层的存储介质读写API相连接。usbd_msc.c、usbd_msc_bot.c、usbd_msc_data.c、usbd_msc_scsi.c属于MSC类的核心文件，其中含有MSC类的设备描述符、配置描述符，并且负责处理由USB主机传来的指令与数据回送。为了实现USB组合设备，所以我们需要将不同USB类的这两种文件分别合并起来，实现我们需要的功能。

## 功能实现

合并完成两种文件后如下图所示，其中只保留了组合类的文件，其余的类文件可以删除。

![USB组合类](https://github.com/YGDL/USB_Composite/raw/main/Photo/USB_Composite.png "USB组合类")