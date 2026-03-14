namespace tasarımdeneme3
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
            pictureBox1 = new PictureBox();
            btnMinimize = new Button();
            panel1 = new Panel();
            btnExit = new Button();
            label1 = new Label();
            label2 = new Label();
            label3 = new Label();
            lblFps = new Label();
            lblSaat = new Label();
            lblDurum = new Label();
            label4 = new Label();
            lblOTONOM = new Label();
            panel2 = new Panel();
            pictureBox2 = new PictureBox();
            panel3 = new Panel();
            pictureBox3 = new PictureBox();
            button1 = new Button();
            textLog = new RichTextBox();
            btnStart = new Button();
            btnOtonom = new Button();
            btnManual = new Button();
            clockTimer = new System.Windows.Forms.Timer(components);
            fpsTimer = new System.Windows.Forms.Timer(components);
            btnSize = new Button();
            button2 = new Button();
            nudMinAngle = new NumericUpDown();
            nudMaxAngle = new NumericUpDown();
            btnAcceptEngagement = new Button();
            btnYarıOto = new Button();
            btn_AtışDevam = new Button();
            BtnSend_Coodinates = new Button();
            label5 = new Label();
            ((System.ComponentModel.ISupportInitialize)pictureBox1).BeginInit();
            panel2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)pictureBox2).BeginInit();
            panel3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)pictureBox3).BeginInit();
            ((System.ComponentModel.ISupportInitialize)nudMinAngle).BeginInit();
            ((System.ComponentModel.ISupportInitialize)nudMaxAngle).BeginInit();
            SuspendLayout();
            // 
            // pictureBox1
            // 
            pictureBox1.BackColor = Color.FromArgb(29, 29, 29);
            pictureBox1.Dock = DockStyle.Fill;
            pictureBox1.Image = Properties.Resources.wallpaper;
            pictureBox1.Location = new Point(0, 0);
            pictureBox1.Name = "pictureBox1";
            pictureBox1.Size = new Size(1600, 900);
            pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox1.TabIndex = 0;
            pictureBox1.TabStop = false;
            pictureBox1.MouseDown += PictureBox1_MouseDown;
            // 
            // btnMinimize
            // 
            btnMinimize.BackgroundImage = Properties.Resources._;
            btnMinimize.FlatAppearance.BorderSize = 0;
            btnMinimize.FlatStyle = FlatStyle.Flat;
            btnMinimize.Location = new Point(1473, 12);
            btnMinimize.Name = "btnMinimize";
            btnMinimize.Size = new Size(30, 29);
            btnMinimize.TabIndex = 3;
            btnMinimize.UseVisualStyleBackColor = true;
            btnMinimize.Click += buttonMinimize_Click;
            // 
            // panel1
            // 
            panel1.AutoSize = true;
            panel1.Dock = DockStyle.Top;
            panel1.Location = new Point(0, 0);
            panel1.Name = "panel1";
            panel1.Size = new Size(1600, 0);
            panel1.TabIndex = 4;
            // 
            // btnExit
            // 
            btnExit.BackgroundImage = Properties.Resources.X;
            btnExit.FlatAppearance.BorderSize = 0;
            btnExit.FlatStyle = FlatStyle.Flat;
            btnExit.Location = new Point(1544, 12);
            btnExit.Name = "btnExit";
            btnExit.Size = new Size(30, 29);
            btnExit.TabIndex = 2;
            btnExit.UseVisualStyleBackColor = true;
            btnExit.Click += button1_Click_1;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.BackColor = Color.FromArgb(29, 29, 29);
            label1.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            label1.ForeColor = Color.White;
            label1.Location = new Point(33, 668);
            label1.Name = "label1";
            label1.Size = new Size(165, 25);
            label1.TabIndex = 5;
            label1.Text = "SİSTEM SAATİ         :";
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.BackColor = Color.FromArgb(29, 29, 29);
            label2.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            label2.ForeColor = Color.White;
            label2.Location = new Point(33, 707);
            label2.Name = "label2";
            label2.Size = new Size(166, 25);
            label2.TabIndex = 6;
            label2.Text = "SİSTEM DURUMU   :";
            // 
            // label3
            // 
            label3.AutoSize = true;
            label3.BackColor = Color.FromArgb(29, 29, 29);
            label3.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            label3.ForeColor = Color.White;
            label3.Location = new Point(715, 808);
            label3.Name = "label3";
            label3.Size = new Size(44, 25);
            label3.TabIndex = 7;
            label3.Text = "FPS:";
            // 
            // lblFps
            // 
            lblFps.AutoSize = true;
            lblFps.BackColor = Color.FromArgb(29, 29, 29);
            lblFps.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            lblFps.ForeColor = Color.White;
            lblFps.Location = new Point(765, 808);
            lblFps.Name = "lblFps";
            lblFps.Size = new Size(30, 25);
            lblFps.TabIndex = 8;
            lblFps.Text = "00";
            // 
            // lblSaat
            // 
            lblSaat.AutoSize = true;
            lblSaat.BackColor = Color.FromArgb(29, 29, 29);
            lblSaat.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            lblSaat.ForeColor = Color.White;
            lblSaat.Location = new Point(194, 668);
            lblSaat.Name = "lblSaat";
            lblSaat.Size = new Size(52, 25);
            lblSaat.TabIndex = 9;
            lblSaat.Text = "00:00";
            // 
            // lblDurum
            // 
            lblDurum.AutoSize = true;
            lblDurum.BackColor = Color.FromArgb(29, 29, 29);
            lblDurum.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            lblDurum.ForeColor = Color.White;
            lblDurum.Location = new Point(195, 707);
            lblDurum.Name = "lblDurum";
            lblDurum.Size = new Size(54, 25);
            lblDurum.TabIndex = 10;
            lblDurum.Text = "PASİF";
            // 
            // label4
            // 
            label4.AutoSize = true;
            label4.BackColor = Color.FromArgb(29, 29, 29);
            label4.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            label4.ForeColor = Color.White;
            label4.Location = new Point(1437, 737);
            label4.Name = "label4";
            label4.Size = new Size(45, 25);
            label4.TabIndex = 11;
            label4.Text = "LOG";
            // 
            // lblOTONOM
            // 
            lblOTONOM.AutoSize = true;
            lblOTONOM.BackColor = Color.FromArgb(29, 29, 29);
            lblOTONOM.Font = new Font("Segoe UI Semilight", 15F, FontStyle.Regular, GraphicsUnit.Point);
            lblOTONOM.ForeColor = Color.White;
            lblOTONOM.Location = new Point(33, 173);
            lblOTONOM.Name = "lblOTONOM";
            lblOTONOM.Size = new Size(442, 35);
            lblOTONOM.TabIndex = 18;
            lblOTONOM.Text = "STNM - HAVA SAVUNMA SİSTEMİ 2025";
            // 
            // panel2
            // 
            panel2.Controls.Add(pictureBox2);
            panel2.Location = new Point(31, 283);
            panel2.Name = "panel2";
            panel2.Size = new Size(640, 360);
            panel2.TabIndex = 21;
            // 
            // pictureBox2
            // 
            pictureBox2.BackColor = Color.FromArgb(29, 29, 29);
            pictureBox2.Cursor = Cursors.Cross;
            pictureBox2.Dock = DockStyle.Fill;
            pictureBox2.Location = new Point(0, 0);
            pictureBox2.Name = "pictureBox2";
            pictureBox2.Size = new Size(640, 360);
            pictureBox2.TabIndex = 22;
            pictureBox2.TabStop = false;
            // 
            // panel3
            // 
            panel3.Controls.Add(pictureBox3);
            panel3.Controls.Add(button1);
            panel3.Location = new Point(696, 283);
            panel3.Name = "panel3";
            panel3.Size = new Size(640, 360);
            panel3.TabIndex = 22;
            // 
            // pictureBox3
            // 
            pictureBox3.BackColor = Color.FromArgb(29, 29, 29);
            pictureBox3.Cursor = Cursors.Cross;
            pictureBox3.Dock = DockStyle.Fill;
            pictureBox3.Location = new Point(0, 0);
            pictureBox3.Name = "pictureBox3";
            pictureBox3.Size = new Size(640, 360);
            pictureBox3.TabIndex = 22;
            pictureBox3.TabStop = false;
            // 
            // button1
            // 
            button1.Font = new Font("Segoe UI", 12F, FontStyle.Bold, GraphicsUnit.Point);
            button1.Location = new Point(173, 331);
            button1.Name = "button1";
            button1.Size = new Size(201, 48);
            button1.TabIndex = 30;
            button1.Text = "Ateş";
            button1.UseVisualStyleBackColor = true;
            button1.Click += button1_Click;
            // 
            // textLog
            // 
            textLog.BackColor = Color.FromArgb(64, 64, 64);
            textLog.BorderStyle = BorderStyle.None;
            textLog.Font = new Font("Segoe UI Semibold", 9F, FontStyle.Bold, GraphicsUnit.Point);
            textLog.ForeColor = Color.White;
            textLog.Location = new Point(1361, 253);
            textLog.Name = "textLog";
            textLog.ScrollBars = RichTextBoxScrollBars.ForcedVertical;
            textLog.Size = new Size(208, 479);
            textLog.TabIndex = 23;
            textLog.Text = "";
            textLog.TextChanged += textLog_TextChanged;
            // 
            // btnStart
            // 
            btnStart.BackgroundImage = Properties.Resources.baslat1;
            btnStart.Cursor = Cursors.Hand;
            btnStart.FlatAppearance.BorderSize = 0;
            btnStart.FlatStyle = FlatStyle.Flat;
            btnStart.Font = new Font("Arial", 21F, FontStyle.Bold, GraphicsUnit.Point);
            btnStart.Location = new Point(1290, 93);
            btnStart.Name = "btnStart";
            btnStart.Size = new Size(257, 67);
            btnStart.TabIndex = 24;
            btnStart.UseVisualStyleBackColor = true;
            btnStart.Click += BtnStart_Click;
            btnStart.MouseLeave += BtnStart_MouseLeave;
            btnStart.MouseMove += BtnStart_MouseMove;
            // 
            // btnOtonom
            // 
            btnOtonom.BackgroundImage = Properties.Resources.otonom1;
            btnOtonom.Cursor = Cursors.Hand;
            btnOtonom.FlatAppearance.BorderSize = 0;
            btnOtonom.FlatStyle = FlatStyle.Flat;
            btnOtonom.Font = new Font("Arial", 21F, FontStyle.Bold, GraphicsUnit.Point);
            btnOtonom.Location = new Point(290, 668);
            btnOtonom.Name = "btnOtonom";
            btnOtonom.Size = new Size(203, 48);
            btnOtonom.TabIndex = 26;
            btnOtonom.UseVisualStyleBackColor = true;
            btnOtonom.Click += BtnOtonom_Click;
            btnOtonom.MouseLeave += BtnOtonom_MouseLeave;
            btnOtonom.MouseMove += BtnOtonom_MouseMove;
            // 
            // btnManual
            // 
            btnManual.BackgroundImage = Properties.Resources.manuel1;
            btnManual.Cursor = Cursors.Hand;
            btnManual.FlatAppearance.BorderSize = 0;
            btnManual.FlatStyle = FlatStyle.Flat;
            btnManual.Font = new Font("Arial", 21F, FontStyle.Bold, GraphicsUnit.Point);
            btnManual.Location = new Point(923, 667);
            btnManual.Name = "btnManual";
            btnManual.Size = new Size(202, 48);
            btnManual.TabIndex = 28;
            btnManual.UseVisualStyleBackColor = true;
            btnManual.Click += BtnManuel_Click;
            btnManual.MouseLeave += BtnManuel_MouseLeave;
            btnManual.MouseMove += BtnManuel_MouseMove;
            // 
            // fpsTimer
            // 
            fpsTimer.Interval = 1000;
            // 
            // btnSize
            // 
            btnSize.BackgroundImage = Properties.Resources.O;
            btnSize.FlatAppearance.BorderSize = 0;
            btnSize.FlatStyle = FlatStyle.Flat;
            btnSize.Location = new Point(1509, 12);
            btnSize.Name = "btnSize";
            btnSize.Size = new Size(30, 29);
            btnSize.TabIndex = 29;
            btnSize.UseVisualStyleBackColor = true;
            btnSize.Click += btnResize_Click;
            // 
            // button2
            // 
            button2.BackColor = SystemColors.Control;
            button2.BackgroundImage = Properties.Resources.Atis3;
            button2.Cursor = Cursors.Hand;
            button2.FlatStyle = FlatStyle.Flat;
            button2.Font = new Font("Segoe UI Semibold", 12F, FontStyle.Bold, GraphicsUnit.Point);
            button2.ForeColor = Color.FromArgb(64, 64, 64);
            button2.Location = new Point(1131, 666);
            button2.Name = "button2";
            button2.Size = new Size(202, 50);
            button2.TabIndex = 28;
            button2.UseVisualStyleBackColor = true;
            button2.Click += button1_Click;
            button2.MouseLeave += button2_MouseLeave;
            button2.MouseMove += button2_MouseMove;
            // 
            // nudMinAngle
            // 
            nudMinAngle.BackColor = Color.FromArgb(64, 64, 64);
            nudMinAngle.ForeColor = Color.White;
            nudMinAngle.Location = new Point(206, 745);
            nudMinAngle.Maximum = new decimal(new int[] { 360, 0, 0, 0 });
            nudMinAngle.Minimum = new decimal(new int[] { 360, 0, 0, int.MinValue });
            nudMinAngle.Name = "nudMinAngle";
            nudMinAngle.Size = new Size(120, 27);
            nudMinAngle.TabIndex = 1;
            nudMinAngle.ValueChanged += nudMinAngle_ValueChanged;
            // 
            // nudMaxAngle
            // 
            nudMaxAngle.BackColor = Color.FromArgb(64, 64, 64);
            nudMaxAngle.ForeColor = Color.White;
            nudMaxAngle.Location = new Point(332, 745);
            nudMaxAngle.Maximum = new decimal(new int[] { 360, 0, 0, 0 });
            nudMaxAngle.Name = "nudMaxAngle";
            nudMaxAngle.Size = new Size(120, 27);
            nudMaxAngle.TabIndex = 2;
            // 
            // btnAcceptEngagement
            // 
            btnAcceptEngagement.BackColor = SystemColors.Control;
            btnAcceptEngagement.BackgroundImage = Properties.Resources.AngajmanKabutEt3;
            btnAcceptEngagement.Cursor = Cursors.Hand;
            btnAcceptEngagement.FlatStyle = FlatStyle.Flat;
            btnAcceptEngagement.Font = new Font("Segoe UI Semibold", 12F, FontStyle.Bold, GraphicsUnit.Point);
            btnAcceptEngagement.ForeColor = Color.FromArgb(64, 64, 64);
            btnAcceptEngagement.Location = new Point(1059, 722);
            btnAcceptEngagement.Name = "btnAcceptEngagement";
            btnAcceptEngagement.Size = new Size(274, 50);
            btnAcceptEngagement.TabIndex = 28;
            btnAcceptEngagement.UseVisualStyleBackColor = true;
            btnAcceptEngagement.Click += BtnAcceptEngagement_Click;
            btnAcceptEngagement.MouseLeave += BtnAcceptEngagement_MouseLeave;
            btnAcceptEngagement.MouseMove += BtnAcceptEngagement_MouseMove;
            // 
            // btnYarıOto
            // 
            btnYarıOto.BackgroundImage = Properties.Resources.yarıotonom1;
            btnYarıOto.Cursor = Cursors.Hand;
            btnYarıOto.FlatAppearance.BorderSize = 0;
            btnYarıOto.FlatStyle = FlatStyle.Flat;
            btnYarıOto.Font = new Font("Segoe UI Semibold", 12F, FontStyle.Bold, GraphicsUnit.Point);
            btnYarıOto.Location = new Point(501, 667);
            btnYarıOto.Name = "btnYarıOto";
            btnYarıOto.Size = new Size(203, 48);
            btnYarıOto.TabIndex = 34;
            btnYarıOto.UseVisualStyleBackColor = true;
            btnYarıOto.Click += BtnYarıOtonom_Click;
            btnYarıOto.MouseLeave += BtnYarıOtonom_MouseLeave;
            btnYarıOto.MouseMove += BtnYarıOtonom_MouseMove;
            // 
            // btn_AtışDevam
            // 
            btn_AtışDevam.BackColor = Color.FromArgb(64, 64, 64);
            btn_AtışDevam.BackgroundImage = Properties.Resources.AtisaDevam3;
            btn_AtışDevam.Cursor = Cursors.Hand;
            btn_AtışDevam.FlatAppearance.BorderSize = 0;
            btn_AtışDevam.FlatStyle = FlatStyle.Flat;
            btn_AtışDevam.Font = new Font("Segoe UI Semibold", 12F, FontStyle.Bold, GraphicsUnit.Point);
            btn_AtışDevam.ForeColor = Color.White;
            btn_AtışDevam.Location = new Point(710, 668);
            btn_AtışDevam.Name = "btn_AtışDevam";
            btn_AtışDevam.Size = new Size(203, 48);
            btn_AtışDevam.TabIndex = 35;
            btn_AtışDevam.UseVisualStyleBackColor = true;
            btn_AtışDevam.Click += btn_AtışDevam_Click;
            btn_AtışDevam.MouseLeave += BtnAtısDevam_MouseLeave;
            btn_AtışDevam.MouseMove += BtnAtısDevam_MouseMove;
            // 
            // BtnSend_Coodinates
            // 
            BtnSend_Coodinates.BackColor = Color.FromArgb(64, 64, 64);
            BtnSend_Coodinates.BackgroundImage = Properties.Resources.add3;
            BtnSend_Coodinates.Cursor = Cursors.Hand;
            BtnSend_Coodinates.FlatStyle = FlatStyle.Flat;
            BtnSend_Coodinates.Font = new Font("Segoe UI Semibold", 12F, FontStyle.Bold, GraphicsUnit.Point);
            BtnSend_Coodinates.ForeColor = Color.FromArgb(64, 64, 64);
            BtnSend_Coodinates.Location = new Point(458, 729);
            BtnSend_Coodinates.Name = "BtnSend_Coodinates";
            BtnSend_Coodinates.Size = new Size(50, 50);
            BtnSend_Coodinates.TabIndex = 28;
            BtnSend_Coodinates.TextAlign = ContentAlignment.MiddleRight;
            BtnSend_Coodinates.UseVisualStyleBackColor = true;
            BtnSend_Coodinates.Click += BtnSend_Coodinates_Click;
            BtnSend_Coodinates.MouseLeave += BtnSendCord_MouseLeave;
            BtnSend_Coodinates.MouseMove += BtnSendCord_MouseMove;
            // 
            // label5
            // 
            label5.AutoSize = true;
            label5.BackColor = Color.FromArgb(29, 29, 29);
            label5.Font = new Font("Segoe UI Semilight", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            label5.ForeColor = Color.White;
            label5.Location = new Point(32, 747);
            label5.Name = "label5";
            label5.Size = new Size(167, 25);
            label5.TabIndex = 36;
            label5.Text = "YASAK AÇI ARALIĞI :";
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1600, 900);
            Controls.Add(label5);
            Controls.Add(BtnSend_Coodinates);
            Controls.Add(btn_AtışDevam);
            Controls.Add(btnYarıOto);
            Controls.Add(nudMinAngle);
            Controls.Add(nudMaxAngle);
            Controls.Add(btnAcceptEngagement);
            Controls.Add(button2);
            Controls.Add(textLog);
            Controls.Add(btnSize);
            Controls.Add(btnManual);
            Controls.Add(btnOtonom);
            Controls.Add(btnStart);
            Controls.Add(panel3);
            Controls.Add(panel2);
            Controls.Add(lblOTONOM);
            Controls.Add(label4);
            Controls.Add(lblDurum);
            Controls.Add(lblSaat);
            Controls.Add(lblFps);
            Controls.Add(label3);
            Controls.Add(label2);
            Controls.Add(label1);
            Controls.Add(btnMinimize);
            Controls.Add(btnExit);
            Controls.Add(panel1);
            Controls.Add(pictureBox1);
            FormBorderStyle = FormBorderStyle.None;
            Name = "Form1";
            Text = "Form1";
            Load += Form1_Load;
            ((System.ComponentModel.ISupportInitialize)pictureBox1).EndInit();
            panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)pictureBox2).EndInit();
            panel3.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)pictureBox3).EndInit();
            ((System.ComponentModel.ISupportInitialize)nudMinAngle).EndInit();
            ((System.ComponentModel.ISupportInitialize)nudMaxAngle).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private PictureBox pictureBox1;
        private Button btnMinimize;
        private Panel panel1;
        private Button btnExit;
        private Label label1;
        private Label label2;
        private Label label3;
        private Label lblFps;
        private Label lblSaat;
        private Label lblDurum;
        private Label label4;
        private Label lblOTONOM;
        private Panel panel2;
        private PictureBox pictureBox2;
        private Panel panel3;
        private PictureBox pictureBox3;
        private RichTextBox textLog;
        private Button btnStart;
        private Button btnOtonom;
        private Button btnManual;
        private System.Windows.Forms.Timer clockTimer;
        private System.Windows.Forms.Timer fpsTimer;
        private Button btnSize;
        private Button button1;
        private Button button2;
        private NumericUpDown nudMinAngle;
        private NumericUpDown nudMaxAngle;
        private Button btnAcceptEngagement;
        private Button btnYarıOto;
        private Button btn_AtışDevam;
        private Button BtnSend_Coodinates;
        private Label label5;
    }
}
