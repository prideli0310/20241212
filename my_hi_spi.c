/*
SPI0 基地址是 0x120C_0000
SPI1 基地址是 0x120C_1000
SPI2 基地址是 0x120C_2000
*/
#define SSP_BASE            (0x120c1000)
/*
SPI1 pinmux
*/
#define SPI1_SCLK           (0x112F0020)
#define SPI1_SDO            (0x112F0024)
#define SPI1_CSN0	        (0x112f0028)
#define SPI1_SDI            (0x112F002C)
 
/*
PERI_CRG111 - SPI Clock Control
*/
#define PERI_CRG111         (0x120101BC)
 
/*
MISC_CTRL0  - SP1_CSN0 Check
*/
#define MISC_CTRL0	         (0x12030000+0x0000)
 
#define SSP_SIZE             0x1000          // 4KB
#define DEFAULT_MD_LEN       (128)
#define IO_ADDRESS_VERIFY(x) (x)
 
 
/* SSP register definition .*/
#define SSP_CR0              IO_ADDRESS_VERIFY(SSP_BASE + 0x00)
#define SSP_CR1              IO_ADDRESS_VERIFY(SSP_BASE + 0x04)
#define SSP_DR               IO_ADDRESS_VERIFY(SSP_BASE + 0x08)
#define SSP_SR               IO_ADDRESS_VERIFY(SSP_BASE + 0x0C)
#define SSP_CPSR             IO_ADDRESS_VERIFY(SSP_BASE + 0x10)
#define SSP_IMSC             IO_ADDRESS_VERIFY(SSP_BASE + 0x14)
#define SSP_RIS              IO_ADDRESS_VERIFY(SSP_BASE + 0x18)
#define SSP_MIS              IO_ADDRESS_VERIFY(SSP_BASE + 0x1C)
#define SSP_ICR              IO_ADDRESS_VERIFY(SSP_BASE + 0x20)
#define SSP_DMACR            IO_ADDRESS_VERIFY(SSP_BASE + 0x24)
#define SSP_SPITXFIFOCR      IO_ADDRESS_VERIFY(SSP_BASE + 0x28)
#define SSP_SPIRXFIFOCR      IO_ADDRESS_VERIFY(SSP_BASE + 0x2C)
 
 
#define SPI_SR_BSY           (0x1 << 4)/* spi busy flag */
#define SPI_SR_TFE           (0x1 << 0)/* Whether to send fifo is empty */
#define SPI_DATA_WIDTH       (9)
#define SPI_SPO              (1)/* 极性 */
#define SPI_SPH              (1)/* 相性 */
#define SPI_SCR              (0)//(8)
#define SPI_CPSDVSR          (2)//(8)
#define SPI_FRAMEMODE        (0)
 
#define MAX_WAIT 10000
//SPI的管脚复用
static void spi1_pinmux()
{
  ssp_writew(SPI1_SCLK, 0x4f1);
  ssp_writew(SPI1_SDO, 0x4f1);
  ssp_writew(SPI1_SDI, 0x4f1);
  spi1_csn0_pinmux();
 
  printf("spi1_pinmux\n");
}
//3 SPI的时钟开启
/* 独立软复位：由寄存器 PERI_CRG111 的 bit[17:15] 控制。相应位写“ 0 ”， SPI 退出软复位；相应位写“ 1 ”， SPI 进入软复位。上电缺省值为 0 。
时钟门控：由寄存器 PERI_CRG111 的 bit[14:12] 控制。相应位写“ 0 ”，关闭时钟；相应位写“ 1 ”，打开时钟。上电缺省值为 1 。

PERI_CRG111的第13bit为SPI1始终们控配置寄存器

    0: 关闭时钟

    1: 打开时钟 */
static void spi1_open_clock()
{
  unsigned int peri_cfg111;
  ssp_readw(PERI_CRG111, peri_cfg111);
  ssp_writew(PERI_CRG111, peri_cfg111|0x1<<13);
  printf("spi1_open_clock\n");
}
//4 SPI1_CSN0启用和配置
/* MISC_CTRL0的第2bit为SPI1信号输出片选选择

    0: SPI1_CSN0

    1: SPI1_CSN1

MISC_CTRL0的第3bit为SPI1片选信号0极性控制

    0: SPI1_CSN0低有效

    1: SPI1_CSN0高有效 */
static void spi1_csn0_pinmux()
{
    	// 复选spi_csn0
	ssp_writew(SPI1_CSN0, 0x0501);
 
	// 配置csn0低有效 且选择片选0
	unsigned int misc_ctrl0;
	ssp_readw(MISC_CTRL0, misc_ctrl0);
	misc_ctrl0 &= ~(0x1<<3); // 低有效
	misc_ctrl0 &= ~(0x1<<2); // 片选0
	ssp_writew(MISC_CTRL0, misc_ctrl0);
}
//5 实现读写寄存器函数
//参考himm和himd
#ifdef __HuaweiLite__
#define  ssp_readw(addr,ret)            (ret =(*(volatile unsigned int *)(addr)))
#define  ssp_writew(addr,value)         ((*(volatile unsigned int *)(addr)) = (value))
#else
static const char dev[]="/dev/mem";
 
static void write_reg(unsigned int Addr, unsigned int Value)
{
    int fd = open (dev, O_RDWR | O_SYNC);
    if (fd < 0)
    {
        printf("open %s error!\n", dev);
        return ;
    }
    
    /* addr align in page_size(4K) */
    unsigned long phy_addr_in_page;
    unsigned long page_diff;
    phy_addr_in_page = Addr & PAGE_SIZE_MASK;
    page_diff = Addr - phy_addr_in_page;
 
    /* size in page_size */
    unsigned long size_in_page;
    unsigned long size = PAGE_SIZE;
    size_in_page =((size + page_diff - 1) & PAGE_SIZE_MASK) + PAGE_SIZE;
    
    void *addr = mmap((void *)0, size_in_page, PROT_READ|PROT_WRITE, MAP_SHARED, fd, phy_addr_in_page);
    if (addr == MAP_FAILED)
    {
        printf("mmap @ 0x%x error!\n", phy_addr_in_page);
        close(fd);
        return;
    }
 
    unsigned int *addr_cur = (unsigned int *)(addr+page_diff);
 
    *addr_cur = Value;
    
    /* munmap */
    if(munmap(addr, size_in_page) != 0 )
    {
        printf("munmap failed!\n");
    }
 
    close(fd);
    //printf("ssp_write_reg %x %d\n", Addr, Value);
}
 
static unsigned int read_reg(unsigned int Addr)
{
    unsigned int Value = 0;
    int fd = open (dev, O_RDWR | O_SYNC);
    if (fd < 0)
    {
        printf("open %s error!\n", dev);
        return -1;
    }
    
    /* addr align in page_size(4K) */
    unsigned long phy_addr_in_page;
    unsigned long page_diff;
    phy_addr_in_page = Addr & PAGE_SIZE_MASK;
    page_diff = Addr - phy_addr_in_page;
 
    /* size in page_size */
    unsigned long size_in_page;
    unsigned long size = PAGE_SIZE;
    size_in_page =((size + page_diff - 1) & PAGE_SIZE_MASK) + PAGE_SIZE;
    
    void *addr = mmap((void *)0, size_in_page, PROT_READ|PROT_WRITE, MAP_SHARED, fd, phy_addr_in_page);
    if (addr == MAP_FAILED)
    {
        printf("mmap @ 0x%x error!\n", phy_addr_in_page);
        close(fd);
        return -1;
    }
 
    unsigned int *addr_cur = (unsigned int *)(addr+page_diff);
 
    Value = *addr_cur;
    
    /* munmap */
    if(munmap(addr, size_in_page) != 0 )
    {
        printf("munmap failed!\n");
    }
 
    close(fd);
    //printf("ssp_read_reg %x %d\n", Addr, Value);
    return Value;
}
 
#define  ssp_readw(addr,ret)            (ret=read_reg(addr))
#define  ssp_writew(addr,value)         (write_reg(addr, value))
#endif

//6 spi开启关闭函数
/* SSP_CR1的第1bit为设置SPI使能

    0: 不使能

    1: 使能 */
static void spi_enable(void)
{
    ssp_writew(SSP_CR1, 0x42);
}
 
static void spi_disable(void)
{
    ssp_writew(SSP_CR1, 0x40);
}
 
void hi_ssp_enable(void)
{
    int ret = 0;
    ssp_readw(SSP_CR1,ret);
    ret = (ret & 0xFFFD) | 0x2;
 
    ret = ret | (0x1 << 4); /* big/little end, 1: little, 0: big */
    ret = ret | (0x1 << 15); /* wait en */
    ssp_writew(SSP_CR1,ret);
}
void hi_ssp_disable(void)
{
    int ret = 0;
    ssp_readw(SSP_CR1,ret);
    ret = ret & (~(0x1 << 1));
    ssp_writew(SSP_CR1,ret);
}
//7 spi帧格式、极性、相性及位宽函数
/* SSP_CR0的[4-5]bit为帧格式选择，这里我们选择了Motorola SPI

    00: Motorla SPI帧格式

    01: TI同步串行格式

    10: National Microwire帧格式

    11: 保留

SSP_CR0的第6bit为SPICLKOUT极性（SPO）

SSP_CR0的第7bit为SPICLKOUT相性（SPH）

SSP_CR0的[0-3]bit为设置数据位宽：

    0011: 4bit

    1000: 9bit

    1101: 14bit

    0100: 5bit

    1001: 10bit */
int hi_ssp_set_frameform(unsigned char framemode,unsigned char spo,unsigned char sph,unsigned char datawidth)
{
    int ret = 0;
    ssp_readw(SSP_CR0,ret);
    if(framemode > 3)
    {
        printf("[ERROR]set frame parameter err.\n");
        return -1;
    }
    ret = (ret & 0xFFCF) | (framemode << 4);
    if((ret & 0x30) == 0)
    {
        if(spo > 1)
        {
            printf("[ERROR]set spo parameter err.\n");
            return -1;
        }
        if(sph > 1)
        {
            printf("[ERROR]set sph parameter err.\n");
            return -1;
        }
        ret = (ret & 0xFF3F) | (sph << 7) | (spo << 6);
    }
    if((datawidth > 16) || (datawidth < 4))
    {
        printf("[ERROR]set datawidth parameter err.\n");
        return -1;
    }
    ret = (ret & 0xFFF0) | (datawidth -1);
    ssp_writew(SSP_CR0,ret);
    return 0;
}
//8 spi串行时钟频率函数
/* 输出 SPI 时钟频率计算方式如下：
Fsspclkout = Fsspclk/ （ CPSDVR x （ 1+SCR ））
Fsspclk ： SPI 的工作参考时钟 100M 。
CPSDVR 、 SCR 请查询相应寄存器。

SPICR0的[8-15]bit为串行时钟率（SCR），取值范围0~255；

SPICPSR的[0-7]bit为时钟分频因子（CPSDVR），此值必须是2~254之间的偶数，也就是0bit位必须为0; */
int hi_ssp_set_serialclock(unsigned char scr,unsigned char cpsdvsr)
{
    int ret = 0;
    ssp_readw(SSP_CR0,ret);
    ret = (ret & 0xFF) | (scr << 8);
    ssp_writew(SSP_CR0,ret);
    if((cpsdvsr & 0x1))
    {
        printf("[ERROR]set cpsdvsr parameter err.\n");
        return -1;
    }
    ssp_writew(SSP_CPSR,cpsdvsr);
    return 0;
}
//9 spi自动手动模式
/* SSP_CR1的第6bit介绍：

    0: 片选信号由芯片逻辑根据所选时序自动产生

    1: 当采用Motorola SPI帧格式时，片选CS信号由SPI使能信号控制，是能后片选拉低，否则片选拉高。 */
int hi_ssp_alt_mode_set(int enable)
{
    int ret = 0;
 
    ssp_readw(SSP_CR1,ret);
    if (enable)
    {
        ret = ret & (~0x40);
    }
    else
    {
        ret = (ret & 0xFF) | 0x40;
    }
    ssp_writew(SSP_CR1,ret);
 
    return 0;
}
//10 spi数据传输函数
static int hi_spi_check_timeout(void)
{
    unsigned int value =  0;
    unsigned int tmp = 0;
    while (1)
    {
        ssp_readw(SSP_SR,value);
        if ((value & SPI_SR_TFE) && (!(value & SPI_SR_BSY)))
        {
            break;
        }
 
        if (tmp++ > MAX_WAIT)
        {
          printf("spi transfer wait timeout!\n");
        	return -1;
        }
        usleep(1);
    }
    return 0;
}
 
void spi_write_a9byte(unsigned char cmd_dat,unsigned char dat)
{
  unsigned short spi_data = 0;
  int ret = 0;
  if(cmd_dat)
  {
      spi_data = 1 << 8;
  }
  else
  {
      spi_data = 0 << 8;
  }
 
  spi_data |= dat;
  //printf("spi_data: %x %x\n", spi_data, dat);
  spi_enable();
  ssp_writew(SSP_DR,spi_data);
  ret =  hi_spi_check_timeout();
  if(ret != 0)
  {
      printf("spi_send timeout\n");
  }
  spi_disable();
}
 
void spi_write_a16byte(unsigned short spi_dat)
{
  int ret = 0;
  spi_enable();
  ssp_writew(SSP_DR, spi_dat);
  //printf("spi_data:0x%x\n",spi_dat);
  ret =  hi_spi_check_timeout();
  if(ret != 0)
  {
      printf("spi_send timeout\n");
  }
  spi_disable();
}
//11 调用例子
void hi_ssp_demo(void)
{
  spi1_pinmux();
  spi1_open_clock();
 
  spi_disable();
  hi_ssp_set_frameform(SPI_FRAMEMODE, SPI_SPO, SPI_SPH, SPI_DATA_WIDTH);
  hi_ssp_set_serialclock(SPI_SCR, SPI_CPSDVSR);
  hi_ssp_alt_mode_set(1);
  hi_ssp_enable();
 
  
  spi_write_a16byte(0x1111);
  spi_write_a16byte(0x1112);
  spi_write_a16byte(0x1113);
  spi_write_a16byte(0x1114);
 
  hi_ssp_disable();
}

/* 总结
1 SPI时钟、数据线、片选线管脚要先复用；

2 SPI模式、极性、相性、位宽、时钟速率要配置好；

3 SPI时钟要开启；我的样例一直在发送数据后检测SR_TFE中阻塞，原因是SPI_SCLK没有开启。

4 片选使能、高低有效位要配置好；

5 如果用于应用层，由于重复打开/dev/mem及mamp，使通讯速度变慢，可以针对性优化 */