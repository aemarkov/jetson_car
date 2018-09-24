namespace RCWin32
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
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
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.txtHost = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.txtPort = new System.Windows.Forms.TextBox();
            this.btnStart = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btnJoystick = new System.Windows.Forms.Button();
            this.btnKeyboard = new System.Windows.Forms.Button();
            this.joystickUpdateTime = new System.Windows.Forms.Timer(this.components);
            this.gboxJoystick = new System.Windows.Forms.GroupBox();
            this.numJoysticId = new System.Windows.Forms.NumericUpDown();
            this.label3 = new System.Windows.Forms.Label();
            this.joystickDrawer = new RCWin32.JoystickDrawer();
            this.gboxKeyboard = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.groupBox1.SuspendLayout();
            this.gboxJoystick.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numJoysticId)).BeginInit();
            this.gboxKeyboard.SuspendLayout();
            this.SuspendLayout();
            // 
            // txtHost
            // 
            this.txtHost.Location = new System.Drawing.Point(40, 19);
            this.txtHost.Name = "txtHost";
            this.txtHost.Size = new System.Drawing.Size(169, 20);
            this.txtHost.TabIndex = 0;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(5, 22);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(29, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Host";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(5, 48);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(26, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "Port";
            // 
            // txtPort
            // 
            this.txtPort.Location = new System.Drawing.Point(40, 45);
            this.txtPort.Name = "txtPort";
            this.txtPort.Size = new System.Drawing.Size(169, 20);
            this.txtPort.TabIndex = 3;
            // 
            // btnStart
            // 
            this.btnStart.Location = new System.Drawing.Point(12, 92);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(215, 23);
            this.btnStart.TabIndex = 4;
            this.btnStart.Text = "Start";
            this.btnStart.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.txtHost);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.txtPort);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Location = new System.Drawing.Point(12, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(215, 74);
            this.groupBox1.TabIndex = 5;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Connection";
            // 
            // btnJoystick
            // 
            this.btnJoystick.Location = new System.Drawing.Point(12, 121);
            this.btnJoystick.Name = "btnJoystick";
            this.btnJoystick.Size = new System.Drawing.Size(108, 23);
            this.btnJoystick.TabIndex = 6;
            this.btnJoystick.Text = "Joystick";
            this.btnJoystick.UseVisualStyleBackColor = true;
            this.btnJoystick.Click += new System.EventHandler(this.btnJoystick_Click);
            // 
            // btnKeyboard
            // 
            this.btnKeyboard.Location = new System.Drawing.Point(126, 121);
            this.btnKeyboard.Name = "btnKeyboard";
            this.btnKeyboard.Size = new System.Drawing.Size(101, 23);
            this.btnKeyboard.TabIndex = 7;
            this.btnKeyboard.Text = "Keyboard";
            this.btnKeyboard.UseVisualStyleBackColor = true;
            this.btnKeyboard.Click += new System.EventHandler(this.button1_Click);
            // 
            // joystickUpdateTime
            // 
            this.joystickUpdateTime.Interval = 10;
            this.joystickUpdateTime.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // gboxJoystick
            // 
            this.gboxJoystick.Controls.Add(this.numJoysticId);
            this.gboxJoystick.Controls.Add(this.label3);
            this.gboxJoystick.Location = new System.Drawing.Point(12, 165);
            this.gboxJoystick.Name = "gboxJoystick";
            this.gboxJoystick.Size = new System.Drawing.Size(215, 88);
            this.gboxJoystick.TabIndex = 8;
            this.gboxJoystick.TabStop = false;
            this.gboxJoystick.Text = "Joystick";
            // 
            // numJoysticId
            // 
            this.numJoysticId.Location = new System.Drawing.Point(6, 42);
            this.numJoysticId.Name = "numJoysticId";
            this.numJoysticId.Size = new System.Drawing.Size(201, 20);
            this.numJoysticId.TabIndex = 5;
            this.numJoysticId.ValueChanged += new System.EventHandler(this.numJoysticId_ValueChanged);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(6, 26);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(201, 13);
            this.label3.TabIndex = 4;
            this.label3.Text = "ID (Sorry, can\'t enumerate. Try 0 for start)";
            // 
            // joystickDrawer
            // 
            this.joystickDrawer.Location = new System.Drawing.Point(12, 259);
            this.joystickDrawer.Name = "joystickDrawer";
            this.joystickDrawer.Rot = 0F;
            this.joystickDrawer.Size = new System.Drawing.Size(215, 215);
            this.joystickDrawer.TabIndex = 6;
            this.joystickDrawer.Text = "joystickDrawer1";
            this.joystickDrawer.Throttle = 0F;
            this.joystickDrawer.X = 0F;
            this.joystickDrawer.Y = 0F;
            // 
            // gboxKeyboard
            // 
            this.gboxKeyboard.Controls.Add(this.label4);
            this.gboxKeyboard.Location = new System.Drawing.Point(12, 165);
            this.gboxKeyboard.Name = "gboxKeyboard";
            this.gboxKeyboard.Size = new System.Drawing.Size(215, 88);
            this.gboxKeyboard.TabIndex = 9;
            this.gboxKeyboard.TabStop = false;
            this.gboxKeyboard.Text = "Keyboard";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 16);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(145, 13);
            this.label4.TabIndex = 0;
            this.label4.Text = "Use WSAD to control the car";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(239, 485);
            this.Controls.Add(this.gboxKeyboard);
            this.Controls.Add(this.joystickDrawer);
            this.Controls.Add(this.gboxJoystick);
            this.Controls.Add(this.btnKeyboard);
            this.Controls.Add(this.btnJoystick);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.btnStart);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.MaximizeBox = false;
            this.Name = "Form1";
            this.Text = "JetsonCar RC";
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.gboxJoystick.ResumeLayout(false);
            this.gboxJoystick.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numJoysticId)).EndInit();
            this.gboxKeyboard.ResumeLayout(false);
            this.gboxKeyboard.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TextBox txtHost;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox txtPort;
        private System.Windows.Forms.Button btnStart;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button btnJoystick;
        private System.Windows.Forms.Button btnKeyboard;
        private System.Windows.Forms.Timer joystickUpdateTime;
        private System.Windows.Forms.GroupBox gboxJoystick;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.NumericUpDown numJoysticId;
        private JoystickDrawer joystickDrawer;
        private System.Windows.Forms.GroupBox gboxKeyboard;
        private System.Windows.Forms.Label label4;
    }
}

