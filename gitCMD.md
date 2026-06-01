# 1. ACP
git add .
git commit -m "comments"
git push -u origin


# 2. 원격 저장소의 최신 변경 사항을 모두 가져옵니다.
git fetch --all

# 3. 로컬 브랜치를 원격 브랜치(예: origin/main) 상태로 강제 초기화합니다.
git reset --hard origin/main

# 4. (선택 사항) Git이 추적하지 않는 파일(untracked files)이나 폴더까지 싹 지우고 싶을 때
git clean -fd