using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace dip_csharp
{
    static class Program
    {
        /// <summary>
        /// app main entry
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
