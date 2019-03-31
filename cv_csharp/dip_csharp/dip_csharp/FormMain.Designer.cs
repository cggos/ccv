namespace dip_csharp
{
    partial class MainForm
    {
        private System.ComponentModel.IContainer components = null;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainForm));
            this.picboxSRC = new System.Windows.Forms.PictureBox();
            this.picboxDST = new System.Windows.Forms.PictureBox();
            this.btnRelief = new System.Windows.Forms.Button();
            this.btnBinary = new System.Windows.Forms.Button();
            this.btnGray = new System.Windows.Forms.Button();
            this.btnGaussBlur = new System.Windows.Forms.Button();
            this.btnLaplaceSharpen = new System.Windows.Forms.Button();
            this.btnInvert = new System.Windows.Forms.Button();
            this.Atomization = new System.Windows.Forms.Button();
            this.btnOil = new System.Windows.Forms.Button();
            this.btnLighting = new System.Windows.Forms.Button();
            this.btnShutter = new System.Windows.Forms.Button();
            this.btnMosaic = new System.Windows.Forms.Button();
            this.btnDistorting = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.picboxSRC)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picboxDST)).BeginInit();
            this.SuspendLayout();
            // 
            // picboxSRC
            // 
            this.picboxSRC.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.picboxSRC.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.picboxSRC.Image = ((System.Drawing.Image)(resources.GetObject("picboxSRC.Image")));
            this.picboxSRC.Location = new System.Drawing.Point(43, 39);
            this.picboxSRC.Name = "picboxSRC";
            this.picboxSRC.Size = new System.Drawing.Size(256, 270);
            this.picboxSRC.SizeMode = System.Windows.Forms.PictureBoxSizeMode.CenterImage;
            this.picboxSRC.TabIndex = 0;
            this.picboxSRC.TabStop = false;
            // 
            // picboxDST
            // 
            this.picboxDST.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.picboxDST.Location = new System.Drawing.Point(323, 39);
            this.picboxDST.Name = "picboxDST";
            this.picboxDST.Size = new System.Drawing.Size(256, 270);
            this.picboxDST.SizeMode = System.Windows.Forms.PictureBoxSizeMode.CenterImage;
            this.picboxDST.TabIndex = 0;
            this.picboxDST.TabStop = false;
            // 
            // btnRelief
            // 
            this.btnRelief.Location = new System.Drawing.Point(43, 339);
            this.btnRelief.Name = "btnRelief";
            this.btnRelief.Size = new System.Drawing.Size(85, 32);
            this.btnRelief.TabIndex = 1;
            this.btnRelief.Text = "Relief";
            this.btnRelief.UseVisualStyleBackColor = true;
            this.btnRelief.Click += new System.EventHandler(this.btnRelief_Click);
            // 
            // btnBinary
            // 
            this.btnBinary.Location = new System.Drawing.Point(134, 339);
            this.btnBinary.Name = "btnBinary";
            this.btnBinary.Size = new System.Drawing.Size(85, 32);
            this.btnBinary.TabIndex = 1;
            this.btnBinary.Text = "Binary";
            this.btnBinary.UseVisualStyleBackColor = true;
            this.btnBinary.Click += new System.EventHandler(this.btnBinary_Click);
            // 
            // btnGray
            // 
            this.btnGray.Location = new System.Drawing.Point(225, 339);
            this.btnGray.Name = "btnGray";
            this.btnGray.Size = new System.Drawing.Size(85, 32);
            this.btnGray.TabIndex = 1;
            this.btnGray.Text = "Gray";
            this.btnGray.UseVisualStyleBackColor = true;
            this.btnGray.Click += new System.EventHandler(this.btnGray_Click);
            // 
            // btnGaussBlur
            // 
            this.btnGaussBlur.Location = new System.Drawing.Point(316, 339);
            this.btnGaussBlur.Name = "btnGaussBlur";
            this.btnGaussBlur.Size = new System.Drawing.Size(85, 32);
            this.btnGaussBlur.TabIndex = 1;
            this.btnGaussBlur.Text = "GaussBlur";
            this.btnGaussBlur.UseVisualStyleBackColor = true;
            this.btnGaussBlur.Click += new System.EventHandler(this.btnGaussBlur_Click);
            // 
            // btnLaplaceSharpen
            // 
            this.btnLaplaceSharpen.Location = new System.Drawing.Point(407, 339);
            this.btnLaplaceSharpen.Name = "btnLaplaceSharpen";
            this.btnLaplaceSharpen.Size = new System.Drawing.Size(85, 32);
            this.btnLaplaceSharpen.TabIndex = 1;
            this.btnLaplaceSharpen.Text = "LaplaceSharpen";
            this.btnLaplaceSharpen.UseVisualStyleBackColor = true;
            this.btnLaplaceSharpen.Click += new System.EventHandler(this.btnLaplaceSharpen_Click);
            // 
            // btnInvert
            // 
            this.btnInvert.Location = new System.Drawing.Point(498, 339);
            this.btnInvert.Name = "btnInvert";
            this.btnInvert.Size = new System.Drawing.Size(85, 32);
            this.btnInvert.TabIndex = 1;
            this.btnInvert.Text = "InvertColor";
            this.btnInvert.UseVisualStyleBackColor = true;
            this.btnInvert.Click += new System.EventHandler(this.btnInvert_Click);
            // 
            // Atomization
            // 
            this.Atomization.Location = new System.Drawing.Point(43, 377);
            this.Atomization.Name = "Atomization";
            this.Atomization.Size = new System.Drawing.Size(85, 32);
            this.Atomization.TabIndex = 1;
            this.Atomization.Text = "Atomization";
            this.Atomization.UseVisualStyleBackColor = true;
            this.Atomization.Click += new System.EventHandler(this.btnAtomization_Click);
            // 
            // btnOil
            // 
            this.btnOil.Location = new System.Drawing.Point(134, 377);
            this.btnOil.Name = "btnOil";
            this.btnOil.Size = new System.Drawing.Size(85, 32);
            this.btnOil.TabIndex = 1;
            this.btnOil.Text = "OilEffect";
            this.btnOil.UseVisualStyleBackColor = true;
            this.btnOil.Click += new System.EventHandler(this.btnOil_Click);
            // 
            // btnLighting
            // 
            this.btnLighting.Location = new System.Drawing.Point(225, 377);
            this.btnLighting.Name = "btnLighting";
            this.btnLighting.Size = new System.Drawing.Size(85, 32);
            this.btnLighting.TabIndex = 1;
            this.btnLighting.Text = "Lighting";
            this.btnLighting.UseVisualStyleBackColor = true;
            this.btnLighting.Click += new System.EventHandler(this.btnLighting_Click);
            // 
            // btnShutter
            // 
            this.btnShutter.Location = new System.Drawing.Point(316, 377);
            this.btnShutter.Name = "btnShutter";
            this.btnShutter.Size = new System.Drawing.Size(85, 32);
            this.btnShutter.TabIndex = 1;
            this.btnShutter.Text = "Shutter";
            this.btnShutter.UseVisualStyleBackColor = true;
            this.btnShutter.Click += new System.EventHandler(this.btnShutter_Click);
            // 
            // btnMosaic
            // 
            this.btnMosaic.Location = new System.Drawing.Point(407, 377);
            this.btnMosaic.Name = "btnMosaic";
            this.btnMosaic.Size = new System.Drawing.Size(85, 32);
            this.btnMosaic.TabIndex = 1;
            this.btnMosaic.Text = "Mosaic";
            this.btnMosaic.UseVisualStyleBackColor = true;
            this.btnMosaic.Click += new System.EventHandler(this.btnMosaic_Click);
            // 
            // btnDistorting
            // 
            this.btnDistorting.Location = new System.Drawing.Point(498, 377);
            this.btnDistorting.Name = "btnDistorting";
            this.btnDistorting.Size = new System.Drawing.Size(85, 32);
            this.btnDistorting.TabIndex = 1;
            this.btnDistorting.Text = "Distorting";
            this.btnDistorting.UseVisualStyleBackColor = true;
            this.btnDistorting.Click += new System.EventHandler(this.btnDistorting_Click);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(621, 437);
            this.Controls.Add(this.btnDistorting);
            this.Controls.Add(this.btnMosaic);
            this.Controls.Add(this.btnShutter);
            this.Controls.Add(this.btnLighting);
            this.Controls.Add(this.btnOil);
            this.Controls.Add(this.Atomization);
            this.Controls.Add(this.btnInvert);
            this.Controls.Add(this.btnLaplaceSharpen);
            this.Controls.Add(this.btnGaussBlur);
            this.Controls.Add(this.btnGray);
            this.Controls.Add(this.btnBinary);
            this.Controls.Add(this.btnRelief);
            this.Controls.Add(this.picboxDST);
            this.Controls.Add(this.picboxSRC);
            this.Name = "MainForm";
            this.Text = "dip_csharp";
            ((System.ComponentModel.ISupportInitialize)(this.picboxSRC)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picboxDST)).EndInit();
            this.ResumeLayout(false);

        }
        
        private System.Windows.Forms.PictureBox picboxSRC;
        private System.Windows.Forms.PictureBox picboxDST;
        private System.Windows.Forms.Button btnRelief;
        private System.Windows.Forms.Button btnBinary;
        private System.Windows.Forms.Button btnGray;
        private System.Windows.Forms.Button btnGaussBlur;
        private System.Windows.Forms.Button btnLaplaceSharpen;
        private System.Windows.Forms.Button btnInvert;
        private System.Windows.Forms.Button Atomization;
        private System.Windows.Forms.Button btnOil;
        private System.Windows.Forms.Button btnLighting;
        private System.Windows.Forms.Button btnShutter;
        private System.Windows.Forms.Button btnMosaic;
        private System.Windows.Forms.Button btnDistorting;
    }
}

