# d2ros

《深入学习ROS 2》

参与贡献请先准备环境，推荐使用Ubuntu22.04。

  * 1、sudo apt update
  * 2、sudo apt upgrade
  * 3、sudo apt install ssh vim git
  * 4、sudo apt install build-essential cmake

安装Python环境

  * 1、sudo apt install python3-pip
  * 2、pip install pip --upgrade
  * 3、pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

安装D2L Book

  * 1、pip install git+https://github.com/d2l-ai/d2l-book
  * 2、sudo apt install pandoc
  * 3、sudo apt install librsvg2-dev
  * 4、sudo apt install texlive-latex-extra texlive-extra-utils texlive-xetex texlive-lang-chinese
  * 5、sudo apt install latexmk
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
