# d2ros

《深入学习ROS 2》

参与贡献请先准备环境，默认推荐使用WSL。

  * 1、控制面板->程序->启用或关闭Windows功能，安装“Hyper-V”、“Windows虚拟机监控程序平台”、“适用于Linux的Windows子系统”、“虚拟机平台”，重启
  * 2、以管理员身份启动命令行，运行：wsl --update；wsl --shutdown
  * 3、打开Microsoft商店，安装Ubuntu22.04
  * 4、进入Ubuntu，设置默认用户名和密码，此处推荐切换为清华源：https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/
  * 5、sudo apt update
  * 6、sudo apt upgrade
  * 7、sudo apt install ssh vim git

安装Python环境

  * 1、wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-py310_24.1.2-0-Linux-x86_64.sh
  * 2、sh Miniconda3-py310_24.1.2-0-Linux-x86_64.sh；bash
  * 3、conda update conda；conda update pip
  * 4、pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

安装D2L Book

  * 1、pip install git+https://github.com/d2l-ai/d2l-book
  * 2、conda install conda-forge::pandoc
  * 3、conda install librsvg
  * 4、sudo apt install build-essential cmake
  * 5、sudo apt install texlive-latex-extra texlive-extra-utils texlive-xetex texlive-lang-chinese
  * 6、sudo apt install latexmk
  * 6、进一步设置和使用请参考：https://book.d2l.ai/

使用Jupyter编辑markdown

  * 1、pip install jupytext
  * 2、重启jupyter：jupyter lab
  * 3、新建：Jupytext->MyST Markdown文件

安装字体

  * 1、sudo apt install fonts-freefont-otf
  * 2、cd /usr/share/fonts/opentype/
  * 3、sudo mkdir d2l-book
  * 4、cd d2l-book
  * 5、sudo cp /path/to/your/fonts/*.otf .
  * 6、sudo chmod 644 *.otf
  * 7、sudo fc-cache -fv
