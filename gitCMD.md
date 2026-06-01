# 1. Add-Commit-Push
git add .
git commit -m "comments"
<!-- 항상 로컬 main → 원격 main -->
git push origin main         
<!-- 현재 체크아웃된 브랜치 -->
git push origin HEAD         
<!-- --set-upstream 으로 지정한 브랜치로 푸시 -->
git push (upstream 설정 후)    


# 2. 원격 저장소의 최신 변경 사항을 모두 가져옵니다.
git fetch --all

# 3. 로컬 브랜치를 원격 브랜치(예: origin/main) 상태로 강제 초기화합니다.
git reset --hard origin/main
