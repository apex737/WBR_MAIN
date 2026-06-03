# 1. Add-Commit-Push
git add .
git commit -m "comments"
<!-- 로컬 브랜치 → main으로 푸시 -->
git push origin HEAD:main         
<!-- 현재 체크아웃된 브랜치에 push -->
git push origin HEAD         
<!-- --set-upstream 으로 지정한 브랜치로 push -->
git push (upstream 설정 후)  
<!-- 임시 브랜치 만들면서 이동  -->  
git switch -c <새로운_브랜치_이름>
<!-- 임시 브랜치 실제로 깃허브에 생성 -->  
git push -u origin [NEW_BRANCH]
<!-- 브랜치 삭제 (-D의 경우 강제삭제) -->  
git branch -d <지울_브랜치_이름>

# 2. 원격 저장소의 최신 변경 사항을 모두 가져옵니다.
git fetch --all

# 3. 로컬 브랜치를 원격 브랜치(예: origin/main) 상태로 강제 초기화합니다.
git reset --hard origin/main
