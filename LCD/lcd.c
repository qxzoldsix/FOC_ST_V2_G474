#include "lcd.h"
#include "lcdfont.h"

/**
************************************************************************
* @brief:      	LCD_Writ_Bus: LCD总线数据写入函数
* @param:      	dat - 要写入的数据
* @retval:     	void
* @details:    	根据硬件配置选择通信方式，支持软件模拟SPI（USE_ANALOG_SPI）和硬件SPI
*               - 开启模拟SPI时：通过GPIO模拟SCLK、MOSI、CS时序，循环写入8位数据
*               - 关闭模拟SPI时：通过HAL_SPI_Transmit使用硬件SPI发送1字节数据
************************************************************************
**/
void LCD_Writ_Bus(uint8_t dat) 
{	
	LCD_CS_Clr();
#if USE_ANALOG_SPI
	for(uint8_t i=0;i<8;i++) {			  
		LCD_SCLK_Clr();
		if(dat&0x80) {
		   LCD_MOSI_Set();
		} else {
		   LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
		dat<<=1;
	}
#else
	HAL_SPI_Transmit(&hspi3, &dat, 1, 0xffff);
#endif	
  LCD_CS_Set();	
}

/**
************************************************************************
* @brief:      	LCD_WR_DATA8: 向LCD写入8位数据
* @param:      	dat - 要写入的8位数据
* @retval:     	void
* @details:    	调用LCD_Writ_Bus函数，将8位数据写入LCD
************************************************************************
**/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

/**
************************************************************************
* @brief:      	LCD_WR_DATA: 向LCD写入16位数据
* @param:      	dat - 要写入的16位数据
* @retval:     	void
* @details:    	拆分16位数据为高8位、低8位，分两次写入LCD
************************************************************************
**/
void LCD_WR_DATA(uint16_t dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}

/**
************************************************************************
* @brief:      	LCD_WR_REG: 向LCD写入寄存器指令
* @param:      	dat - 要写入的寄存器指令
* @retval:     	void
* @details:    	拉低DC引脚表示写入指令，完成后拉高DC引脚
************************************************************************
**/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//写入指令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写入数据
}

/**
************************************************************************
* @brief:      	LCD_Address_Set: 设置LCD显示区域坐标
* @param:      	x1, y1, x2, y2 - 显示区域的起始和结束坐标
* @retval:     	void
* @details:    	根据屏幕方向，通过指令设置LCD列地址、行地址，进入写入模式
************************************************************************
**/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+52);
		LCD_WR_DATA(x2+52);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+40);
		LCD_WR_DATA(y2+40);
		LCD_WR_REG(0x2c);//显存写入
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+53);
		LCD_WR_DATA(x2+53);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+40);
		LCD_WR_DATA(y2+40);
		LCD_WR_REG(0x2c);//显存写入
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+40);
		LCD_WR_DATA(x2+40);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+53);
		LCD_WR_DATA(y2+53);
		LCD_WR_REG(0x2c);//显存写入
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+40);
		LCD_WR_DATA(x2+40);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+52);
		LCD_WR_DATA(y2+52);
		LCD_WR_REG(0x2c);//显存写入
	}
}

/**
************************************************************************
* @brief:      	LCD_Fill: LCD指定区域颜色填充
* @param:      	xsta, ysta, xend, yend - 填充区域对角坐标
*              	color - 填充颜色
* @retval:     	void
* @details:    	设置显示区域后，循环写入颜色值完成填充
************************************************************************
**/
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/**
************************************************************************
* @brief:      	LCD_DrawPoint: 在LCD指定位置画点
* @param:      	x, y - 坐标位置
*              	color - 点颜色
* @retval:     	void
* @details:    	设置单点坐标，写入颜色值，完成画点
************************************************************************
**/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//设置单点坐标
	LCD_WR_DATA(color);
} 

/**
************************************************************************
* @brief:      	LCD_DrawLine: 在LCD画直线
* @param:      	x1, y1 - 起点坐标
*              	x2, y2 - 终点坐标
*              	color - 线条颜色
* @retval:     	void
* @details:    	使用Bresenham算法，根据坐标增量绘制任意方向直线
************************************************************************
**/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置步进方向 
	else if (delta_x==0)incx=0;//竖线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取最大增量 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/**
************************************************************************
* @brief:      	LCD_DrawRectangle: 在LCD画矩形
* @param:      	x1, y1 - 矩形左上角坐标
*              	x2, y2 - 矩形右下角坐标
*              	color - 矩形颜色
* @retval:     	void
* @details:    	调用画线函数，绘制矩形四条边框
************************************************************************
**/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}

/**
************************************************************************
* @brief:      	Draw_Circle: 在LCD画圆
* @param:      	x0, y0 - 圆心坐标
*              	r - 圆半径
*              	color - 圆颜色
* @retval:     	void
* @details:    	使用中点画圆算法，绘制8对称点完成圆形绘制
************************************************************************
**/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断半径是否需要调整
		{
			b--;
		}
	}
}

/**
************************************************************************
* @brief:      	LCD_ShowChinese: 在LCD显示汉字字符串
* @param:      	x, y - 起始显示坐标
*              	s - 汉字字符串指针
*              	fc - 前景颜色
*              	bc - 背景颜色
*              	sizey - 字号：12/16/24/32
*              	mode - 显示模式：1=透明叠加 0=覆盖显示
* @retval:     	void
* @details:    	根据字号选择对应字库，循环显示汉字
************************************************************************
**/
void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/**
************************************************************************
* @brief:      	LCD_ShowChinese12x12: 显示12x12汉字
* @param:      	x, y - 坐标
*              	s - 汉字编码指针
*              	fc - 前景色
*              	bc - 背景色
*              	sizey - 字号
*              	mode - 显示模式
* @retval:     	void
************************************************************************
**/
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数量
	uint16_t TypefaceNum;//一个汉字占用字节数
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字个数
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //找到对应汉字后退出循环
	}
} 

/**
************************************************************************
* @brief:      	LCD_ShowChinese16x16: 显示16x16汉字
************************************************************************
**/
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数量
	uint16_t TypefaceNum;//一个汉字占用字节数
	uint16_t x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字个数
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //找到对应汉字后退出循环
	}
} 

/**
************************************************************************
* @brief:      	LCD_ShowChinese24x24: 显示24x24汉字
************************************************************************
**/
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数量
	uint16_t TypefaceNum;//一个汉字占用字节数
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字个数
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //找到对应汉字后退出循环
	}
} 

/**
************************************************************************
* @brief:      	LCD_ShowChinese32x32: 显示32x32汉字
************************************************************************
**/
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数量
	uint16_t TypefaceNum;//一个汉字占用字节数
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字个数
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //找到对应汉字后退出循环
	}
}

/**
************************************************************************
* @brief:      	LCD_ShowChar: 显示单个ASCII字符
* @param:      	x,y - 坐标
*              	num - ASCII字符
*              	fc - 前景色
*              	bc - 背景色
*              	sizey - 字号
*              	mode - 显示模式
* @retval:     	void
************************************************************************
**/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//一个字符占用字节数
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //偏移计算
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置坐标
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/**
************************************************************************
* @brief:      	LCD_ShowString: 显示ASCII字符串
* @param:      	x,y - 坐标
*              	p - 字符串指针
*              	fc - 前景色
*              	bc - 背景色
*              	sizey - 字号
*              	mode - 显示模式
* @retval:     	void
************************************************************************
**/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

/**
************************************************************************
* @brief:      	mypow: 计算m的n次方
* @param:      	m - 底数
*              	n - 指数
* @retval:     	uint32_t - 计算结果
************************************************************************
**/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}

/**
************************************************************************
* @brief:      	LCD_ShowIntNum: 显示整数
* @param:      	x,y - 坐标
*              	num - 整数值
*              	len - 显示位数
*              	fc - 前景色
*              	bc - 背景色
*              	sizey - 字号
* @retval:     	void
************************************************************************
**/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

/**
************************************************************************
* @brief:      	LCD_ShowFloatNum: 显示浮点数（带符号）
* @param:      	x,y - 坐标
*              	num - 浮点数
*              	len - 整数位数
*              	decimal - 小数位数
*              	fc - 前景色
*              	bc - 背景色
*              	sizey - 字号
* @retval:     	void
************************************************************************
**/
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
{
		int16_t num_int;
		uint8_t t, temp, sizex;
    sizex = sizey / 2;
    num_int = num * mypow(10, decimal);

    if (num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        num_int = -num_int;
        x += sizex;
        len++;
    }
    else
    {
        LCD_ShowChar(x, y, ' ', fc, bc, sizey, 0);
        num_int = num_int;
        x += sizex;
        len++;
    }

    // 刷新显示区域
    LCD_Fill(x, y, x + len * sizex + decimal + 1, y + sizey + 1, bc);

    for (t = 0; t < len; t++)
    {
        if (t == (len - decimal))
        {
            LCD_ShowChar(x + (len - decimal) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        temp = ((num_int / mypow(10, len - t - 1)) % 10) + '0';
        LCD_ShowChar(x + t * sizex, y, temp, fc, bc, sizey, 0);
    }
}

/**
************************************************************************
* @brief:      	LCD_ShowFloatNum1: 显示浮点数（无符号对齐版）
* @param:      	x,y - 坐标
*              	num - 浮点数
*              	len - 整数位数
*              	decimal - 小数位数
*              	fc - 前景色
*              	bc - 背景色
*              	sizey - 字号
* @retval:     	void
************************************************************************
**/
 void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey)
  {
      uint8_t t, temp, sizex, enshow = 0;
      sizex = sizey / 2;

      if (num < 0) num = -num;
      uint32_t num_int = (uint32_t)(num * mypow(10, decimal) + 0.5f);


      uint32_t int_part = num_int / mypow(10, decimal);
      uint8_t int_width = 0;
      uint32_t tmp = int_part;
      if (tmp == 0) int_width = 1;
      else { while (tmp) { int_width++; tmp /= 10; } }
      if (int_width > len) int_width = len;

      LCD_Fill(x, y, x + (len + 1 + decimal) * sizex, y + sizey, bc);

      //整数位显示
      for (t = 0; t < len; t++)
      {
          temp = (num_int / mypow(10, len + decimal - t - 1)) % 10;
          if (enshow == 0 && t < (len - int_width))
          {
              LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
              continue;
          }
          enshow = 1;
          LCD_ShowChar(x + t * sizex, y, temp + '0', fc, bc, sizey, 0);
      }

      //小数点
      LCD_ShowChar(x + len * sizex, y, '.', fc, bc, sizey, 0);

      //小数位
      for (t = 0; t < decimal; t++)
      {
          temp = (num_int / mypow(10, decimal - t - 1)) % 10;
          LCD_ShowChar(x + (len + 1 + t) * sizex, y, temp + '0', fc, bc, sizey, 0);
      }
  }

/**
************************************************************************
* @brief:      	LCD_ShowPicture: 显示图片
* @param:      	x,y - 起始坐标
*              	length - 图片宽度
*              	width - 图片高度
*              	pic[] - 图片数据数组
* @retval:     	void
************************************************************************
**/
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}

/**
************************************************************************
* @brief:      	LCD_Init: LCD初始化函数
* @param:      	void
* @retval:     	void
* @details:    	执行硬件复位、背光开启、寄存器配置、默认界面绘制
************************************************************************
**/
void LCD_Init(void)
{
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	LCD_RES_Clr();//复位
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
	
	LCD_BLK_Set();//打开背光
  HAL_Delay(100);
	
	LCD_WR_REG(0x11); 
	HAL_Delay(120); 
	LCD_WR_REG(0x36); 
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33); 

	LCD_WR_REG(0xB7); 
	LCD_WR_DATA8(0x35);  

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x19);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x12);   

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20);  

	LCD_WR_REG(0xC6); 
	LCD_WR_DATA8(0x0F);    

	LCD_WR_REG(0xD0); 
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2B);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4C);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x51);
	LCD_WR_DATA8(0x2F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29); 
		LCD_Fill(0,0,LCD_W,LCD_H,BLACK);	
		LCD_Fill(0, 0, LCD_W, 21, DARKBLUE);
		LCD_ShowString(3, 0, (uint8_t *)"noFoc", WHITE, DARKBLUE, 24, 0);
		LCD_ShowString(75, 5, (uint8_t *)"READY", GREEN, DARKBLUE, 12, 0);
		LCD_ShowString(135, 5, (uint8_t *)"Vbus:", GRAY, DARKBLUE, 12, 0);
		LCD_ShowString(222, 5, (uint8_t *)"V", GRAY, DARKBLUE, 12, 0);

		LCD_Fill(0, 22, LCD_W, 42, LGRAY);
		LCD_DrawLine(0, 21, LCD_W, 21, GRAYBLUE);
		LCD_ShowString(5, 24, (uint8_t *)"Freq:", WHITE, LGRAY, 12, 0);
		LCD_ShowString(80, 24, (uint8_t *)"Hz", GRAY, LGRAY, 12, 0);
		LCD_DrawLine(105, 24, 105, 38, GRAYBLUE);
		LCD_ShowString(115, 24, (uint8_t *)"Volt:", WHITE, LGRAY, 12, 0);
		LCD_ShowString(195, 24, (uint8_t *)"pu", GRAY, LGRAY, 12, 0);

		LCD_Fill(0, 42, LCD_W, 43, GRAYBLUE);
		LCD_ShowString(5, 45, (uint8_t *)"Mode:", WHITE, BLACK, 12, 0);
		LCD_ShowString(50, 45, (uint8_t *)" POS ", CYAN, BLACK, 12, 0);

		LCD_DrawLine(0, 57, LCD_W, 57, GRAYBLUE);
		LCD_ShowString(5,  60, (uint8_t *)"Iq",   WHITE,  BLACK, 12, 0);
		LCD_ShowString(5,  78, (uint8_t *)"Vel",  WHITE,  BLACK, 12, 0);
		LCD_ShowString(5,  96, (uint8_t *)"Pos",  WHITE,  BLACK, 12, 0);
		LCD_ShowString(5, 114, (uint8_t *)"Tor",  WHITE,  BLACK, 12, 0);

		LCD_DrawLine(42,  59, 42,  129, GRAYBLUE);
		LCD_DrawLine(160, 59, 160, 129, GRAYBLUE);
		LCD_ShowString(96, 60, (uint8_t *)"->",     GRAY,   BLACK, 12, 0);
		LCD_ShowString(96, 78, (uint8_t *)"->",     GRAY,   BLACK, 12, 0);
		LCD_ShowString(96, 96, (uint8_t *)"->",     GRAY,   BLACK, 12, 0);
		LCD_ShowString(96,114, (uint8_t *)"->",     GRAY,   BLACK, 12, 0);
		LCD_ShowString(168,60, (uint8_t *)"A",      GRAY,   BLACK, 12, 0);
		LCD_ShowString(168,78, (uint8_t *)"r/s",    GRAY,   BLACK, 12, 0);
		LCD_ShowString(168,96, (uint8_t *)"rad",    GRAY,   BLACK, 12, 0);
		LCD_ShowString(168,114,(uint8_t *)"N/m",    GRAY,   BLACK, 12, 0);

		LCD_DrawLine(0, 130, LCD_W, 130, GRAYBLUE);
		
}