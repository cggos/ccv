using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace dip_csharp
{
    public partial class MainForm : Form
    {
        ImageProcess procImg;

        public MainForm()
        {
            InitializeComponent();

            procImg = new ImageProcess(this.picboxSRC.Image);
        }

        private void btnRelief_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Relief();
        }

        private void btnBinary_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Binary();
        }

        private void btnGray_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Gray();
        }

        private void btnGaussBlur_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.GaussBlur();
        }

        private void btnLaplaceSharpen_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.LaplaceSharpen();
        }

        private void btnInvert_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Invert();
        }

        private void btnAtomization_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Atomization();
        }

        private void btnOil_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Oil();
        }

        private void btnLighting_Click(object sender, EventArgs e)
        {
            this.picboxDST.Image = procImg.Lighting();
        }

        private void btnShutter_Click(object sender, EventArgs e)
        {
            procImg.Shutter(this.picboxSRC, this.picboxDST, 1);
        }

        private void btnMosaic_Click(object sender, EventArgs e)
        {
            procImg.Mosaic(this.picboxSRC, this.picboxDST);
        }

        private void btnDistorting_Click(object sender, EventArgs e)
        {
            procImg.Distorting(this.picboxSRC, this.picboxDST);
        }
    }
}
