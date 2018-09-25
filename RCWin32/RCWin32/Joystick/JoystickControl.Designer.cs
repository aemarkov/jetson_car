namespace RCWin32.Joystick
{
    partial class JoystickControl
    {
        /// <summary> 
        /// Обязательная переменная конструктора.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Освободить все используемые ресурсы.
        /// </summary>
        /// <param name="disposing">истинно, если управляемый ресурс должен быть удален; иначе ложно.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Код, автоматически созданный конструктором компонентов

        /// <summary> 
        /// Требуемый метод для поддержки конструктора — не изменяйте 
        /// содержимое этого метода с помощью редактора кода.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.gboxKeyboard = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.joystickDrawer = new RCWin32.Joystick.JoystickDrawer();
            this.btnKeyboard = new System.Windows.Forms.Button();
            this.btnJoystick = new System.Windows.Forms.Button();
            this.gboxJoystick = new System.Windows.Forms.GroupBox();
            this.numJoysticId = new System.Windows.Forms.NumericUpDown();
            this.label3 = new System.Windows.Forms.Label();
            this.joystickUpdateTimer = new System.Windows.Forms.Timer(this.components);
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.gboxKeyboard.SuspendLayout();
            this.gboxJoystick.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numJoysticId)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // gboxKeyboard
            // 
            this.gboxKeyboard.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.gboxKeyboard.Controls.Add(this.label4);
            this.gboxKeyboard.Location = new System.Drawing.Point(6, 48);
            this.gboxKeyboard.Name = "gboxKeyboard";
            this.gboxKeyboard.Size = new System.Drawing.Size(215, 88);
            this.gboxKeyboard.TabIndex = 13;
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
            // joystickDrawer
            // 
            this.joystickDrawer.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.joystickDrawer.Location = new System.Drawing.Point(6, 142);
            this.joystickDrawer.Name = "joystickDrawer";
            this.joystickDrawer.Rot = 0F;
            this.joystickDrawer.Size = new System.Drawing.Size(215, 215);
            this.joystickDrawer.TabIndex = 10;
            this.joystickDrawer.Text = "joystickDrawer1";
            this.joystickDrawer.Throttle = 0F;
            this.joystickDrawer.X = 0F;
            this.joystickDrawer.Y = 0F;
            // 
            // btnKeyboard
            // 
            this.btnKeyboard.Location = new System.Drawing.Point(115, 19);
            this.btnKeyboard.Name = "btnKeyboard";
            this.btnKeyboard.Size = new System.Drawing.Size(106, 23);
            this.btnKeyboard.TabIndex = 12;
            this.btnKeyboard.Text = "Keyboard";
            this.btnKeyboard.UseVisualStyleBackColor = true;
            this.btnKeyboard.Click += new System.EventHandler(this.btnKeyboard_Click);
            // 
            // btnJoystick
            // 
            this.btnJoystick.Location = new System.Drawing.Point(6, 19);
            this.btnJoystick.Name = "btnJoystick";
            this.btnJoystick.Size = new System.Drawing.Size(104, 23);
            this.btnJoystick.TabIndex = 11;
            this.btnJoystick.Text = "Joystick";
            this.btnJoystick.UseVisualStyleBackColor = true;
            this.btnJoystick.Click += new System.EventHandler(this.btnJoystick_Click);
            // 
            // gboxJoystick
            // 
            this.gboxJoystick.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.gboxJoystick.Controls.Add(this.numJoysticId);
            this.gboxJoystick.Controls.Add(this.label3);
            this.gboxJoystick.Location = new System.Drawing.Point(6, 48);
            this.gboxJoystick.Name = "gboxJoystick";
            this.gboxJoystick.Size = new System.Drawing.Size(215, 88);
            this.gboxJoystick.TabIndex = 9;
            this.gboxJoystick.TabStop = false;
            this.gboxJoystick.Text = "Joystick";
            // 
            // numJoysticId
            // 
            this.numJoysticId.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
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
            // joystickUpdateTimer
            // 
            this.joystickUpdateTimer.Interval = 10;
            this.joystickUpdateTimer.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // groupBox1
            // 
            this.groupBox1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBox1.Controls.Add(this.gboxJoystick);
            this.groupBox1.Controls.Add(this.gboxKeyboard);
            this.groupBox1.Controls.Add(this.btnKeyboard);
            this.groupBox1.Controls.Add(this.btnJoystick);
            this.groupBox1.Controls.Add(this.joystickDrawer);
            this.groupBox1.Location = new System.Drawing.Point(0, 0);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(227, 364);
            this.groupBox1.TabIndex = 14;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Control";
            // 
            // JoystickControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBox1);
            this.Name = "JoystickControl";
            this.Size = new System.Drawing.Size(227, 364);
            this.gboxKeyboard.ResumeLayout(false);
            this.gboxKeyboard.PerformLayout();
            this.gboxJoystick.ResumeLayout(false);
            this.gboxJoystick.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numJoysticId)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox gboxKeyboard;
        private System.Windows.Forms.Label label4;
        private JoystickDrawer joystickDrawer;
        private System.Windows.Forms.Button btnKeyboard;
        private System.Windows.Forms.Button btnJoystick;
        private System.Windows.Forms.GroupBox gboxJoystick;
        private System.Windows.Forms.NumericUpDown numJoysticId;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Timer joystickUpdateTimer;
        private System.Windows.Forms.GroupBox groupBox1;
    }
}
