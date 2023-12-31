/* FSMC相关参数 定义 
* 注意: 我们默认是通过FSMC块3来连接SRAM, 块1有4个片选: FSMC_NE1~4 * 
* 修改SRAM_FSMC_NEX, 对应的SRAM_CS_GPIO相关设置也得改 */ 
#define SRAM_FSMC_NEX 3 
/* 使用FSMC_NE3接SRAM_CS,取值范围只能是: 1~4 */ 
/*****************************************************************/ 
/* SRAM基地址, 根据 SRAM_FSMC_NEX 的设置来决定基址地址 
* 我们一般使用FSMC的块1(BANK1)来驱动SRAM, 块1地址范围总大小为256MB,均分成4块: 
* 存储块1(FSMC_NE1)地址范围: 0X6000 0000 ~ 0X63FF FFFF 
* 存储块2(FSMC_NE2)地址范围: 0X6400 0000 ~ 0X67FF FFFF 
* 存储块3(FSMC_NE3)地址范围: 0X6800 0000 ~ 0X6BFF FFFF 
* 存储块4(FSMC_NE4)地址范围: 0X6C00 0000 ~ 0X6FFF FFFF */ 
#define SRAM_BASE_ADDR (0x60000000 + (SRAM_FSMC_NEX - 1) * 0x4000000)