cd .. 

## convco 根据 git-commit 记录 来 推测 下个 升级版本
$version = $(convco version --bump)

## 生成该版本的日志
convco changelog > CHANGELOG.md

## 将 更新日志的 Unreleased 替换成 最新版本
(Get-Content CHANGELOG.md) -replace 'Unreleased', "v$version" | Set-Content CHANGELOG.md

# 提交 CHANGELOG.md
git add .
git commit -m "docs: $version CHANGELOG.md"

# 用指定版本，发布 到 Github && crates.io
# 注：管道 前面要 输出 y 给 crago release，免得 后面运行的时候，需要人工输入，无法自动化
Write-Output y | cargo release --execute $version

pause