
一、 基础开发（每天都会用）

这些命令构成了 Git 的最基本工作流：
```
git status：查看当前状态。看看哪些文件改动了，哪些还没提交。

git add <文件名>：将文件添加到暂存区。如果是 git add . 则添加所有改动。

git commit -m "说明文字"：将暂存区的内容提交到本地仓库。

git log --oneline：查看简单的提交历史。

git diff：查看当前代码和上次提交的代码具体哪里不一样。
```
二、 分支管理（多任务并行）

在 ROS 开发中，通常会在不同分支测试不同功能：
```
git branch：查看当前所有分支。

git checkout -b <新分支名>：创建并切换到一个新分支。

git checkout <分支名>：切换到已有的分支。

git merge <分支名>：将指定分支的代码合并到当前分支。

git branch -d <分支名>：删除已合并的分支。
```
三、 撤销与“后悔药”（最重要）

如果你写乱了或者想找回之前的代码：
```
git checkout -- <文件名>：丢弃本地未提交的改动，把文件恢复到上次提交的状态。

git reset HEAD~1：撤销最后一次 commit，但保留你写的代码改动（回到了 git add 之前的状态）。

git reset --hard <CommitID>：强制回滚到某个历史版本，那个版本之后的所有代码都会丢失。

git stash：临时把改动藏起来。当你代码写到一半想紧急修复另一个 bug，但又不想提交时很有用。用 git stash pop 恢复。
```
四、 远程仓库同步（与 Gitee/GitHub 通信）
```
git clone <地址>：克隆远程仓库到本地。

git pull：拉取远程代码并自动合并到本地。

git fetch：只拉取远程代码，不合并（更安全，让你先看看对方改了啥）。

git push：把本地提交的代码推送到服务器。

git remote -v：查看远程仓库的地址。

git remote add upstream <地址>：关联原作者的仓库（用于同步更新）。
```
五、 处理子模块（你之前编译 acados 遇到的）

很多复杂的 ROS 项目带嵌套仓库：
```
git submodule update --init --recursive：初始化并下载所有子模块（如果你 clone 完发现文件夹是空的，必执行此条）。
```
解决nav_params.yaml冲突问题：
按顺序执行：
1）只看远程会动你哪些行（不改动工作区）
```
git fetch origin
git diff HEAD..origin/master -- src/my_navigation/nav_bringup/config/nav_params.yaml
```
2）把当前未提交修改收起来
```
git stash push -m "wip nav_params" -- src/my_navigation/nav_bringup/config/nav_params.yaml
```
（若不止这一个文件，可去掉 -- 路径 整仓 stash。）

3）合并远程
```
git merge origin/master
```
或用习惯写法：
```
git pull origin master
```
4）把本地改动拿回来，在文件里自己决定保留

```
git stash pop
```