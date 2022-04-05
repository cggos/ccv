using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Windows.Forms;

namespace dip_csharp
{
    class ImageProcess
    {
        int widthImgSrc ;
        int heightImgSrc;
        Bitmap oldBitmap;

        public ImageProcess(Image imgSrc)
        {
            widthImgSrc = imgSrc.Width;
            heightImgSrc = imgSrc.Height;

            oldBitmap = (Bitmap)imgSrc;
        }

        /// <summary>
        /// 图像灰度化：
        /// 原理: 彩色图像处理成黑白效果通常有3种算法；
        ///(1).最大值法: 使每个像素点的 R, G, B 值等于原像素点的 RGB (颜色值) 中最大的一个；
        ///(2).平均值法: 使用每个像素点的 R,G,B值等于原像素点的RGB值的平均值；
        ///(3).加权平均值法: 对每个像素点的 R, G, B值进行加权
        /// ---自认为第三种方法做出来的黑白效果图像最 "真实"
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Gray(int nType = 2)
        {
            Bitmap newBitmap = new Bitmap(widthImgSrc, heightImgSrc);
            Color pixel;
            for (int x = 0; x < widthImgSrc; x++)
            {
                for (int y = 0; y < heightImgSrc; y++)
                {
                    pixel = oldBitmap.GetPixel(x, y);
                    int r, g, b, Result = 0;
                    r = pixel.R;
                    g = pixel.G;
                    b = pixel.B;
                    //实例程序以加权平均值法产生黑白图像
                    switch (nType)
                    {
                        case 0://平均值法
                            Result = ((r + g + b) / 3);
                            break;
                        case 1://最大值法
                            Result = r > g ? r : g;
                            Result = Result > b ? Result : b;
                            break;
                        case 2://加权平均值法
                            Result = ((int)(0.7 * r) + (int)(0.2 * g) + (int)(0.1 * b));
                            break;
                    }
                    newBitmap.SetPixel(x, y, Color.FromArgb(Result, Result, Result));
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 图像二值化
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Binary()
        {
            int i, j, iAvg, iPixel;
            Color oldColor, newColor;
            RectangleF rect = new RectangleF(0, 0, widthImgSrc, heightImgSrc);
            Bitmap newBitmap = oldBitmap.Clone(rect, System.Drawing.Imaging.PixelFormat.DontCare);
            i = 0;
            while (i < widthImgSrc - 1)
            {
                j = 0;
                while (j < heightImgSrc - 1)
                {
                    oldColor = newBitmap.GetPixel(i, j);
                    iAvg = (oldColor.R + oldColor.G + oldColor.B) / 3;
                    iPixel = 0;
                    if (iAvg >= 128)
                        iPixel = 255;
                    else
                        iPixel = 0;
                    newColor = Color.FromArgb(255, iPixel, iPixel, iPixel);
                    newBitmap.SetPixel(i, j, newColor);
                    j = j + 1;
                }
                i = i + 1;
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 反色效果，底片效果
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Invert()
        {
            Bitmap newBitmap = new Bitmap(widthImgSrc, heightImgSrc);
            Color pixel;
            for (int x = 1; x < widthImgSrc; x++)
            {
                for (int y = 1; y < heightImgSrc; y++)
                {
                    int r, g, b;
                    pixel = oldBitmap.GetPixel(x, y);
                    r = 255 - pixel.R;
                    g = 255 - pixel.G;
                    b = 255 - pixel.B;
                    newBitmap.SetPixel(x, y, Color.FromArgb(r, g, b));
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 高斯模糊，柔化效果
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image GaussBlur()
        {
            Bitmap newBitmap = new Bitmap(widthImgSrc, heightImgSrc);
            Color pixel;
            //高斯模板
            int[] Gauss = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };
            for (int x = 1; x < widthImgSrc - 1; x++)
            {
                for (int y = 1; y < heightImgSrc - 1; y++)
                {
                    int r = 0, g = 0, b = 0;
                    int nIndex = 0;
                    for (int row = -1; row <= 1; row++)
                    {
                        for (int col = -1; col <= 1; col++)
                        {
                            pixel = oldBitmap.GetPixel(x + row, y + col);
                            r += pixel.R * Gauss[nIndex];
                            g += pixel.G * Gauss[nIndex];
                            b += pixel.B * Gauss[nIndex];
                            nIndex++;
                        }
                    }
                    r /= 16;
                    g /= 16;
                    b /= 16;
                    //处理颜色值溢出
                    r = r > 255 ? 255 : r;
                    r = r < 0 ? 0 : r;
                    g = g > 255 ? 255 : g;
                    g = g < 0 ? 0 : g;
                    b = b > 255 ? 255 : b;
                    b = b < 0 ? 0 : b;
                    newBitmap.SetPixel(x - 1, y - 1, Color.FromArgb(r, g, b));
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 拉普拉斯边缘锐化
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image LaplaceSharpen()
        {
            Bitmap newBitmap = new Bitmap(widthImgSrc, heightImgSrc);
            Color pixel;
            //拉普拉斯模板
            int[] Laplacian = { -1, -1, -1, -1, 9, -1, -1, -1, -1 };
            for (int x = 1; x < widthImgSrc - 1; x++)
            {
                for (int y = 1; y < heightImgSrc - 1; y++)
                {
                    int r = 0, g = 0, b = 0;
                    int Index = 0;
                    for (int col = -1; col <= 1; col++)
                        for (int row = -1; row <= 1; row++)
                        {
                            pixel = oldBitmap.GetPixel(x + row, y + col); r += pixel.R * Laplacian[Index];
                            g += pixel.G * Laplacian[Index];
                            b += pixel.B * Laplacian[Index];
                            Index++;
                        }
                    //处理颜色值溢出
                    r = r > 255 ? 255 : r;
                    r = r < 0 ? 0 : r;
                    g = g > 255 ? 255 : g;
                    g = g < 0 ? 0 : g;
                    b = b > 255 ? 255 : b;
                    b = b < 0 ? 0 : b;
                    newBitmap.SetPixel(x - 1, y - 1, Color.FromArgb(r, g, b));
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 浮雕效果,原理: 对图像像素点的像素值分别与相邻像素点的像素值相减后加上128, 然后将其作为新的像素点的值.
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Relief()
        {
            Bitmap newBitmap = new Bitmap(widthImgSrc, heightImgSrc);
            Color pixel1, pixel2;
            for (int x = 0; x < widthImgSrc - 1; x++)
            {
                for (int y = 0; y < heightImgSrc - 1; y++)
                {
                    int r = 0, g = 0, b = 0;
                    pixel1 = oldBitmap.GetPixel(x, y);
                    pixel2 = oldBitmap.GetPixel(x + 1, y + 1);
                    r = Math.Abs(pixel1.R - pixel2.R + 128);
                    g = Math.Abs(pixel1.G - pixel2.G + 128);
                    b = Math.Abs(pixel1.B - pixel2.B + 128);
                    if (r > 255)
                        r = 255;
                    if (r < 0)
                        r = 0;
                    if (g > 255)
                        g = 255;
                    if (g < 0)
                        g = 0;
                    if (b > 255)
                        b = 255;
                    if (b < 0)
                        b = 0;
                    newBitmap.SetPixel(x, y, Color.FromArgb(r, g, b));
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 雾化效果，原理: 在图像中引入一定的随机值, 打乱图像中的像素值
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Atomization()
        {
            Bitmap newBitmap = new Bitmap(widthImgSrc, heightImgSrc);
            Color pixel;
            for (int x = 1; x < widthImgSrc - 1; x++)
            {
                for (int y = 1; y < heightImgSrc - 1; y++)
                {
                    System.Random MyRandom = new Random();
                    int k = MyRandom.Next(123456);
                    //像素块大小
                    int dx = x + k % 19;
                    int dy = y + k % 19;
                    if (dx >= widthImgSrc)
                        dx = widthImgSrc - 1;
                    if (dy >= heightImgSrc)
                        dy = heightImgSrc - 1;
                    pixel = oldBitmap.GetPixel(dx, dy);
                    newBitmap.SetPixel(x, y, pixel);
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 以光照效果，原理: 对图像中的某一范围内的像素的亮度分别进行处理.
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Lighting()
        {
            Bitmap newBitmap = oldBitmap.Clone(new RectangleF(0, 0, widthImgSrc, heightImgSrc), System.Drawing.Imaging.PixelFormat.DontCare);
            int A = widthImgSrc / 2;
            int B = heightImgSrc / 2;
            //MyCenter图片中心点，发亮此值会让强光中心发生偏移
            Point MyCenter = new Point(widthImgSrc / 2, heightImgSrc / 2);
            //R强光照射面的半径，即”光晕”
            int R = Math.Min(widthImgSrc / 2, heightImgSrc / 2);
            for (int i = widthImgSrc - 1; i >= 1; i--)
            {
                for (int j = heightImgSrc - 1; j >= 1; j--)
                {
                    float MyLength = (float)Math.Sqrt(Math.Pow((i - MyCenter.X), 2) + Math.Pow((j - MyCenter.Y), 2));
                    //如果像素位于”光晕”之内
                    if (MyLength < R)
                    {
                        Color MyColor = newBitmap.GetPixel(i, j);
                        int r, g, b;
                        //220亮度增加常量，该值越大，光亮度越强
                        float MyPixel = 220.0f * (1.0f - MyLength / R);
                        r = MyColor.R + (int)MyPixel;
                        r = Math.Max(0, Math.Min(r, 255));
                        g = MyColor.G + (int)MyPixel;
                        g = Math.Max(0, Math.Min(g, 255));
                        b = MyColor.B + (int)MyPixel;
                        b = Math.Max(0, Math.Min(b, 255));
                        //将增亮后的像素值回写到位图
                        Color MyNewColor = Color.FromArgb(255, r, g, b);
                        newBitmap.SetPixel(i, j, MyNewColor);
                    }
                }
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }

        /// <summary>
        /// 油画效果，原理：对图像中某一范围内的像素引入随机值
        /// </summary>
        /// <param name="imgSrc"></param>
        /// <returns></returns>
        public Image Oil()
        {
            //以油画效果显示图像
            RectangleF rect = new RectangleF(0, 0, widthImgSrc, heightImgSrc);
            Bitmap newBitmap = oldBitmap.Clone(rect, System.Drawing.Imaging.PixelFormat.DontCare);
            //产生随机数序列
            Random rnd = new Random();
            //取不同的值决定油画效果的不同程度
            int iModel = 2;
            int i = widthImgSrc - iModel;
            while (i > 1)
            {
                int j = heightImgSrc - iModel;
                while (j > 1)
                {
                    int iPos = rnd.Next(100000) % iModel;
                    //将该点的RGB值设置成附近iModel点之内的任一点
                    Color color = newBitmap.GetPixel(i + iPos, j + iPos);
                    newBitmap.SetPixel(i, j, color);
                    j = j - 1;
                }
                i = i - 1;
            }
            Image imgDst;
            imgDst = newBitmap;
            return imgDst;
        }



        //////////////////////////////////////////////////////////////////////////

        /// <summary>
        /// 垂直、水平百叶窗
        /// </summary>
        /// <param name="picboxSrc"></param>
        /// <param name="picboxDst"></param>
        /// <param name="nType"></param>
        public void Shutter(PictureBox picboxSrc, PictureBox picboxDst,int nType = 0)
        {
            //垂直百叶窗显示图像
            //根据窗口或图像的高度或宽度和定制的百叶窗显示条宽度计算百叶窗显示的条数量 ；
            //根据窗口或图像的高度或宽度定制百叶窗显示条数量计算百窗显示的条宽度.
            if (nType == 0)
            {
                Bitmap MyBitmap = (Bitmap)picboxSrc.Image.Clone();
                int dw = MyBitmap.Width / 30;
                int dh = MyBitmap.Height;
                Graphics g = picboxDst.CreateGraphics();
                g.Clear(Color.Gray);
                Point[] MyPoint = new Point[30];
                for (int x = 0; x < 30; x++)
                {
                    MyPoint[x].Y = 0;
                    MyPoint[x].X = x * dw;
                }
                Bitmap bitmap = new Bitmap(MyBitmap.Width, MyBitmap.Height);
                for (int i = 0; i < dw; i++)
                {
                    for (int j = 0; j < 30; j++)
                    {
                        for (int k = 0; k < dh; k++)
                        {
                            bitmap.SetPixel(MyPoint[j].X + i, MyPoint[j].Y + k,
                            MyBitmap.GetPixel(MyPoint[j].X + i, MyPoint[j].Y + k));
                        }
                    }
                    picboxDst.Refresh();
                    picboxDst.Image = bitmap;
                    System.Threading.Thread.Sleep(100);
                }
            }

            //水平百叶窗显示图像
            if (nType == 1)
            {
                Bitmap MyBitmap = (Bitmap)picboxSrc.Image.Clone();
                int dh = MyBitmap.Height / 20;
                int dw = MyBitmap.Width;
                Graphics g = picboxDst.CreateGraphics();
                g.Clear(Color.Gray);
                Point[] MyPoint = new Point[20];
                for (int y = 0; y < 20; y++)
                {
                    MyPoint[y].X = 0;
                    MyPoint[y].Y = y * dh;
                }
                Bitmap bitmap = new Bitmap(MyBitmap.Width, MyBitmap.Height);
                for (int i = 0; i < dh; i++)
                {
                    for (int j = 0; j < 20; j++)
                    {
                        for (int k = 0; k < dw; k++)
                        {
                            bitmap.SetPixel(MyPoint[j].X + k, MyPoint[j].Y + i, MyBitmap.GetPixel(MyPoint[j].X + k, MyPoint[j].Y + i));
                        }
                    }
                    picboxDst.Refresh();
                    picboxDst.Image = bitmap;
                    System.Threading.Thread.Sleep(100);
                }
            }
        }

        /// <summary>
        /// 马赛克效果
        /// 原理: 确定图像的随机位置点和确定马赛克块的大小,然后马赛克块图像覆盖随机点即可.
        /// </summary>
        /// <param name="picboxSrc"></param>
        /// <param name="picboxDst"></param>
        public void Mosaic(PictureBox picboxSrc, PictureBox picboxDst)
        {
            Bitmap MyBitmap = (Bitmap)picboxSrc.Image.Clone();
            int dw = MyBitmap.Width / 50;
            int dh = MyBitmap.Height / 50;
            Graphics g = picboxDst.CreateGraphics();
            g.Clear(Color.Gray);
            Point[] MyPoint = new Point[2500];
            for (int x = 0; x < 50; x++)
                for (int y = 0; y < 50; y++)
                {
                    MyPoint[x * 50 + y].X = x * dw;
                    MyPoint[x * 50 + y].Y = y * dh;
                }
            Bitmap bitmap = new Bitmap(MyBitmap.Width, MyBitmap.Height);
            for (int i = 0; i < 10000; i++)
            {
                System.Random MyRandom = new Random();
                int iPos = MyRandom.Next(2500);
                for (int m = 0; m < dw; m++)
                    for (int n = 0; n < dh; n++)
                    {
                        bitmap.SetPixel(MyPoint[iPos].X + m, MyPoint[iPos].Y + n, MyBitmap.GetPixel(MyPoint[iPos].X + m, MyPoint[iPos].Y + n));
                    }
                picboxDst.Refresh();
                picboxDst.Image = bitmap;
            }
            for (int i = 0; i < 2500; i++)
                for (int m = 0; m < dw; m++)
                    for (int n = 0; n < dh; n++)
                    {
                        bitmap.SetPixel(MyPoint[i].X + m, MyPoint[i].Y + n, MyBitmap.GetPixel(MyPoint[i].X + m, MyPoint[i].Y + n));
                    }
            picboxDst.Refresh();
            picboxDst.Image = bitmap;
        }
        
        /// <summary>
        /// 扭曲效果
        /// </summary>
        /// <param name="picboxSrc"></param>
        /// <param name="picboxDst"></param>
        public void Distorting(PictureBox picboxSrc, PictureBox picboxDst)
        {
            Bitmap MyBitmap = (Bitmap)picboxSrc.Image.Clone();
            int w = MyBitmap.Width;
            int h = MyBitmap.Height;
            w = 20;
            h = 20;
            if (h == picboxDst.Height / 2)
            {
                w = 0;
                h = 0;
            }
            Size offset = new Size(w++, h++);//设置偏移量
            Graphics g = picboxDst.CreateGraphics();
            Rectangle rect = picboxDst.ClientRectangle;
            Point[] points = new Point[3];
            points[0] = new Point(rect.Left + offset.Width, rect.Top + offset.Height);
            points[1] = new Point(rect.Right, rect.Top + offset.Height);
            points[2] = new Point(rect.Left, rect.Bottom - offset.Height);
            g.Clear(Color.White);
            g.DrawImage(MyBitmap, points);
        }
    }
}