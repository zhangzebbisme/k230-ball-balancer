@echo off
cd /d C:\Users\Zz200\Desktop\k230-git-tmp
git config user.name "zhangzebbisme"
git config user.email "zhangzebbisme@users.noreply.github.com"
git add -A
git commit -m "Initial commit: K230 ball balancing control with UART multi-task + Wi-Fi MJPEG stream"
git remote add origin https://github.com/zhangzebbisme/k230-ball-balancer.git
set GIT_TERMINAL_PROMPT=0
git push -u origin main
