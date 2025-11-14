# 팀 프로젝트를 위한 Git & GitHub 가이드
**대학교 4인 팀 프로젝트용 (Ubuntu 환경)**

---

## 목차
1. [소개](#1-소개)
2. [초기 설정](#2-초기-설정)
3. [프로젝트 시작하기](#3-프로젝트-시작하기)
4. [기본 Git 작업](#4-기본-git-작업)
5. [브랜치(Branch) 작업](#5-브랜치branch-작업)
6. [Merge Conflict 해결](#6-merge-conflict-해결)
7. [VS Code에서 Git 사용하기](#7-vs-code에서-git-사용하기)
8. [팀 프로젝트 워크플로우 제안](#8-팀-프로젝트-워크플로우-제안)
9. [자주 하는 실수와 해결법](#9-자주-하는-실수와-해결법)
10. [부록: Pull Request 사용하기](#부록-pull-requestpr-사용하기)

---

## 1. 소개

이 문서는 4명이 함께하는 대학교 팀 프로젝트를 위한 Git과 GitHub 사용 가이드입니다. 팀원들이 효율적으로 협업하고 코드를 관리할 수 있도록 필수적인 기능들을 중심으로 설명합니다.

### 1.1 Git과 GitHub란?

**Git**은 코드의 변경 이력을 관리하는 버전 관리 시스템입니다. 누가, 언제, 무엇을, 왜 변경했는지 추적할 수 있습니다.

**GitHub**는 Git 저장소를 온라인에서 호스팅하고 팀원들과 공유할 수 있는 플랫폼입니다. 협업을 위한 다양한 기능을 제공합니다.

### 1.2 이 문서에서 다루는 내용

- Git 설치 및 초기 설정
- 저장소 생성 및 Clone
- 기본 Git 명령어 (commit, push, pull)
- 브랜치 생성 및 관리
- Merge Conflict 해결 방법
- VS Code에서 Git 사용하기
- 팀 프로젝트 워크플로우
- Pull Request 사용법 (부록)

---

## 2. 초기 설정

### 2.1 Git 설치 (Ubuntu)

터미널을 열고 다음 명령어를 실행하세요:

```bash
# 패키지 목록 업데이트
sudo apt update

# Git 설치
sudo apt install git

# 설치 확인
git --version
```

설치가 완료되면 `git version 2.x.x` 같은 버전 정보가 출력됩니다.

### 2.2 Git 초기 설정

Git을 처음 사용하기 전에 사용자 정보를 설정해야 합니다. 이 정보는 모든 commit에 기록됩니다.

터미널에서 다음 명령어를 실행하세요:

```bash
git config --global user.name "홍길동"
git config --global user.email "your.email@example.com"
```

설정 확인:

```bash
git config --global --list
```

> 💡 **팁**: 이메일 주소는 GitHub 계정과 동일하게 설정하는 것이 좋습니다. 그래야 GitHub에서 내 commit을 정확히 표시할 수 있습니다.

### 2.3 GitHub 계정 생성

1. https://github.com 접속
2. 'Sign up' 클릭하여 계정 생성
3. 이메일 인증 완료
4. 프로필 설정 (선택사항)

### 2.4 VS Code 설치 및 Git 확장

VS Code는 Git을 기본적으로 지원하며, 추가 확장을 설치하면 더 편리하게 사용할 수 있습니다.

**VS Code 설치 (Ubuntu):**

```bash
# Snap을 이용한 설치 (권장)
sudo snap install --classic code

# 또는 공식 저장소를 통한 설치
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg
sudo apt update
sudo apt install code
```

**유용한 VS Code 확장 프로그램 설치 (선택사항):**

1. VS Code 실행 후 좌측의 Extensions 아이콘 클릭 (또는 `Ctrl+Shift+X`)
2. 다음 확장 프로그램 검색 및 설치:
   - **GitLens** - Git 기록을 시각적으로 보여주는 강력한 도구
   - **Git Graph** - 브랜치와 commit 이력을 그래프로 표시

---

## 3. 프로젝트 시작하기

### 3.1 GitHub에 저장소 생성 (팀장이 수행)

1. GitHub에 로그인
2. 우측 상단의 '+' 아이콘 클릭 → 'New repository' 선택
3. Repository 정보 입력:
   - **Repository name**: 프로젝트 이름 (예: team-project)
   - **Description**: 프로젝트 설명 (선택사항)
   - **Public/Private**: Public (공개) 또는 Private (비공개) 선택
   - **Initialize with README**: 체크 (README.md 파일 자동 생성)
4. 'Create repository' 클릭
5. Settings → Collaborators → 'Add people' 클릭하여 팀원들 초대

> 💡 **팁**: Private 저장소를 만들면 초대받은 팀원만 접근할 수 있습니다. 학교 프로젝트는 보통 Private로 설정합니다.

### 3.2 저장소 Clone하기

Clone은 GitHub의 저장소를 내 컴퓨터로 복사하는 작업입니다. 모든 팀원이 수행해야 합니다.

#### 방법 1: 터미널에서 Clone

1. GitHub 저장소 페이지에서 녹색 'Code' 버튼 클릭
2. HTTPS 탭에서 URL 복사 (예: `https://github.com/username/team-project.git`)
3. 터미널을 열고 프로젝트를 저장할 폴더로 이동
4. 다음 명령어 실행:

```bash
cd ~/Projects  # 원하는 위치로 이동
git clone https://github.com/username/team-project.git
cd team-project
```

#### 방법 2: VS Code에서 Clone

1. VS Code 실행
2. `Ctrl+Shift+P` 눌러 명령 팔레트 열기
3. 'Git: Clone' 입력 후 선택
4. 저장소 URL 입력
5. 저장할 폴더 선택

> 💡 **팁**: Clone 후 VS Code에서 'Open Folder'로 해당 폴더를 열면 Git 기능을 바로 사용할 수 있습니다.

---

## 4. 기본 Git 작업

Git의 기본 워크플로우는 다음과 같습니다: **파일 수정 → Stage → Commit → Push**

### 4.1 변경사항 확인 (git status)

현재 작업 디렉토리의 상태를 확인합니다. 어떤 파일이 수정되었는지, 어떤 파일이 staged 되었는지 알 수 있습니다.

```bash
git status
```

이 명령어는 자주 사용하게 됩니다. commit 전에 항상 확인하는 습관을 들이세요.

### 4.2 파일 Stage하기 (git add)

Stage는 commit할 파일을 선택하는 단계입니다. 모든 변경사항을 한 번에 commit할 필요는 없으며, 관련된 변경사항끼리 묶어서 commit하는 것이 좋습니다.

**특정 파일만 Stage:**
```bash
git add filename.txt
```

**여러 파일 Stage:**
```bash
git add file1.txt file2.txt
```

**모든 변경사항 Stage:**
```bash
git add .
```

> ⚠️ **주의**: `git add .`는 모든 파일을 stage하므로 의도하지 않은 파일이 포함될 수 있습니다. 중요한 commit 전에는 `git status`로 확인하세요.

### 4.3 Commit하기 (git commit)

Commit은 staged된 변경사항을 로컬 저장소에 기록하는 작업입니다. 각 commit에는 변경 내용을 설명하는 메시지를 작성해야 합니다.

```bash
git commit -m "로그인 기능 구현"
```

#### 좋은 Commit 메시지 작성 가이드:

- **명확하고 간결하게**: 무엇을 했는지 한눈에 알 수 있도록
- **동사로 시작**: '추가', '수정', '삭제', '구현' 등
- **구체적으로**: 'fix bug' 보다는 '로그인 시 에러 메시지 수정'

**좋은 예시:**
- "회원가입 API 연동 완료"
- "메인 페이지 레이아웃 수정"
- "데이터베이스 연결 오류 수정"

**나쁜 예시:**
- "수정"
- "버그 픽스"
- "작업중"

### 4.4 GitHub에 Push하기 (git push)

Push는 로컬 저장소의 commit들을 GitHub 저장소에 업로드하는 작업입니다. Push를 해야 팀원들이 내 변경사항을 볼 수 있습니다.

```bash
git push origin main
```

여기서 `origin`은 원격 저장소를 가리키며, `main`은 브랜치 이름입니다. (저장소에 따라 'master'일 수도 있습니다)

> 💡 **팁**: 처음 push할 때 GitHub 로그인을 요구할 수 있습니다. 사용자 이름과 비밀번호(또는 Personal Access Token)를 입력하세요.

### 4.5 GitHub에서 Pull하기 (git pull)

Pull은 GitHub 저장소의 최신 변경사항을 내 로컬 저장소로 가져오는 작업입니다. 팀원들이 push한 내용을 받아오려면 pull을 해야 합니다.

```bash
git pull origin main
```

> ⚠️ **중요**: 작업을 시작하기 전에 항상 pull을 먼저 실행하세요! 최신 코드가 아닌 상태에서 작업하면 나중에 merge conflict가 발생할 가능성이 높아집니다.

### 4.6 기본 작업 워크플로우 정리

**매일 작업 시작 시:**
1. `git pull origin main` (최신 코드 받아오기)
2. 작업 시작

**작업 완료 후:**
1. `git status` (변경사항 확인)
2. `git add .` (또는 특정 파일)
3. `git commit -m "작업 내용"`
4. `git pull origin main` (push 전 최신 코드 확인)
5. `git push origin main`

---

## 5. 브랜치(Branch) 작업

브랜치는 독립적인 작업 공간을 만들어주는 기능입니다. 여러 기능을 동시에 개발하거나, 실험적인 코드를 작성할 때 유용합니다.

### 5.1 브랜치의 개념

main 브랜치는 프로젝트의 기본 브랜치로, 안정적인 코드가 유지되어야 합니다. 새로운 기능을 개발할 때는 별도의 브랜치를 만들어 작업한 후, 완료되면 main 브랜치에 합치는 방식으로 작업합니다.

**브랜치 사용의 장점:**
- main 브랜치를 안전하게 유지할 수 있음
- 여러 기능을 동시에 개발 가능
- 팀원들 간의 작업 충돌을 줄일 수 있음
- 실험적인 코드를 안전하게 테스트 가능

### 5.2 브랜치 확인하기

**현재 브랜치 목록 보기:**
```bash
git branch
```

**원격 저장소 포함 모든 브랜치 보기:**
```bash
git branch -a
```

현재 작업 중인 브랜치는 '*' 표시로 표시됩니다.

### 5.3 새 브랜치 생성하기

새 브랜치를 만드는 명령어:

```bash
git branch feature/login
```

**브랜치 이름 규칙:**
- `feature/기능명`: 새로운 기능 개발 (예: feature/login)
- `fix/버그명`: 버그 수정 (예: fix/button-error)
- `개인이름/작업명`: 개인 작업 (예: hong/ui-design)

> 💡 **팁**: 브랜치 이름은 소문자와 하이픈(-)을 사용하며, 무엇을 작업하는지 명확히 알 수 있도록 짓는 것이 좋습니다.

### 5.4 브랜치 이동하기 (git checkout)

다른 브랜치로 이동하는 명령어입니다. checkout을 하면 작업 디렉토리가 해당 브랜치의 상태로 변경됩니다.

```bash
git checkout feature/login
```

**브랜치를 생성하면서 동시에 이동하기 (자주 사용):**
```bash
git checkout -b feature/login
```

> ⚠️ **주의**: 브랜치를 이동하기 전에 현재 작업을 commit하거나 stash해야 합니다. 그렇지 않으면 변경사항이 사라질 수 있습니다.

### 5.5 브랜치에서 작업하기

브랜치에서의 작업은 main 브랜치와 동일합니다:

1. 파일 수정
2. `git add .`
3. `git commit -m "로그인 UI 구현"`
4. `git push origin feature/login`

주의: push할 때 브랜치 이름을 정확히 입력해야 합니다!

### 5.6 브랜치 병합하기 (git merge)

작업이 완료된 브랜치를 main 브랜치에 합치는 작업입니다.

```bash
# 1. main 브랜치로 이동
git checkout main

# 2. 최신 상태로 업데이트
git pull origin main

# 3. 브랜치 병합
git merge feature/login

# 4. GitHub에 push
git push origin main
```

> 💡 **팁**: 작은 팀 프로젝트에서는 직접 merge하지 않고, Pull Request를 통해 merge하는 것이 더 좋습니다. (부록 참조)

### 5.7 브랜치 삭제하기

병합이 완료되어 더 이상 필요없는 브랜치는 삭제할 수 있습니다.

**로컬 브랜치 삭제:**
```bash
git branch -d feature/login
```

**원격 브랜치 삭제:**
```bash
git push origin --delete feature/login
```

---

## 6. Merge Conflict 해결

Merge conflict(병합 충돌)는 두 명 이상의 팀원이 같은 파일의 같은 부분을 수정했을 때 발생합니다. Git이 자동으로 병합할 수 없어 수동으로 해결해야 합니다.

### 6.1 Conflict가 발생하는 상황

**예시 상황:**
1. 팀원 A가 `app.py`의 10번째 줄을 수정하고 push
2. 팀원 B가 같은 `app.py`의 10번째 줄을 다르게 수정
3. 팀원 B가 pull 또는 merge를 시도 → Conflict 발생!

> 💡 **팁**: Conflict는 자연스러운 현상입니다. 당황하지 말고 차근차근 해결하면 됩니다!

### 6.2 Conflict 확인하기

Conflict가 발생하면 다음과 같은 메시지가 나타납니다:

```
CONFLICT (content): Merge conflict in app.py
Automatic merge failed; fix conflicts and then commit the result.
```

**어떤 파일에서 conflict가 발생했는지 확인:**
```bash
git status
```

### 6.3 Conflict 해결하기

Conflict가 발생한 파일을 열면 다음과 같은 표시가 있습니다:

```python
<<<<<<< HEAD
현재 브랜치의 코드
=======
병합하려는 브랜치의 코드
>>>>>>> feature/login
```

**해결 방법:**

1. 충돌이 발생한 파일을 에디터로 엽니다
2. `<<<<<<<`, `=======`, `>>>>>>>` 표시를 확인합니다
3. 어떤 코드를 유지할지 결정합니다:
   - 현재 브랜치 코드만 유지
   - 병합하려는 브랜치 코드만 유지
   - 둘 다 유지하거나 새로운 코드로 작성
4. `<<<<<<<`, `=======`, `>>>>>>>` 표시를 모두 삭제합니다
5. 파일을 저장합니다

#### 해결 예시

**충돌 발생 전:**
```python
def greet():
    print('Hello')
```

**충돌 발생 후 (파일 내용):**
```python
def greet():
<<<<<<< HEAD
    print('안녕하세요')
=======
    print('Hello, World!')
>>>>>>> feature/greeting
```

**해결 후 (둘 다 유지하기로 결정):**
```python
def greet():
    print('안녕하세요')
    print('Hello, World!')
```

### 6.4 해결 완료 후 작업

충돌을 해결한 후에는 다음 단계를 진행합니다:

```bash
# 1. 해결한 파일을 stage
git add app.py

# 2. Commit (merge conflict 해결 commit)
git commit -m "Merge conflict 해결"

# 3. Push
git push origin main
```

> ⚠️ **중요**: 충돌 해결 후에는 반드시 코드가 제대로 작동하는지 테스트하세요! 자동 병합이 실패했다는 것은 코드의 동작도 영향을 받았을 가능성이 높습니다.

### 6.5 Conflict 예방하기

- 작업 시작 전 항상 pull하기
- 자주 commit하고 push하기 (큰 변경사항을 오래 보관하지 않기)
- 팀원들과 작업 영역 나누기 (같은 파일을 동시에 수정하지 않기)
- 브랜치를 활용하여 독립적으로 작업하기
- 팀원들과 소통하여 작업 진행 상황 공유하기

---

## 7. VS Code에서 Git 사용하기

VS Code는 Git을 시각적으로 쉽게 사용할 수 있는 인터페이스를 제공합니다. 터미널 명령어를 외우지 않아도 대부분의 Git 작업을 수행할 수 있습니다.

### 7.1 Source Control 패널 열기

- 좌측 사이드바에서 Source Control 아이콘 클릭 (브랜치 모양)
- 또는 단축키: `Ctrl+Shift+G`

### 7.2 변경사항 확인하기

Source Control 패널에서 수정된 파일 목록을 볼 수 있습니다:

- **U** (Untracked): 새로 생성된 파일
- **M** (Modified): 수정된 파일
- **D** (Deleted): 삭제된 파일

파일을 클릭하면 변경 내용을 비교하여 볼 수 있습니다 (diff 뷰).

### 7.3 파일 Stage하기 (git add)

Source Control 패널에서 Changes 섹션을 확인합니다.

**방법 1: 개별 파일 Stage**
- 파일 옆의 '+' 버튼 클릭

**방법 2: 모든 파일 Stage**
- Changes 옆의 '+' 버튼 클릭

### 7.4 Commit하기 (git commit)

1. Source Control 패널 상단의 메시지 입력 상자에 commit 메시지 입력
2. `Ctrl+Enter` 또는 상단의 '✓' 버튼 클릭

> 💡 **팁**: 여러 줄의 commit 메시지를 작성하려면 메시지 입력 후 `Shift+Enter`를 누르세요.

### 7.5 Push & Pull

**Push (git push):**
- Source Control 패널 상단의 '...' 메뉴 → 'Push' 선택

**Pull (git pull):**
- Source Control 패널 상단의 '...' 메뉴 → 'Pull' 선택

**단축 버튼:**
하단 상태바의 브랜치 이름 옆에 있는 동기화 버튼(↻)을 클릭하면 pull과 push를 동시에 수행합니다.

### 7.6 브랜치 작업

**현재 브랜치 확인:**
- 하단 상태바 좌측에 현재 브랜치 이름이 표시됩니다

**브랜치 생성 및 전환:**
1. 하단 상태바의 브랜치 이름 클릭
2. 상단에 나타나는 메뉴에서:
   - 다른 브랜치 선택 → 해당 브랜치로 checkout
   - 'Create new branch...' 선택 → 새 브랜치 생성

### 7.7 Merge Conflict 해결

VS Code는 merge conflict를 시각적으로 해결할 수 있는 도구를 제공합니다.

1. Conflict가 발생하면 Source Control 패널에 'Merge Changes' 섹션이 나타남
2. 충돌 파일 클릭하여 열기
3. 파일 내에서 충돌 부분 위에 나타나는 버튼 사용:
   - **Accept Current Change**: 현재 브랜치 코드 유지
   - **Accept Incoming Change**: 병합하려는 브랜치 코드 유지
   - **Accept Both Changes**: 둘 다 유지
   - **Compare Changes**: 변경 사항 비교
4. 충돌 해결 후 파일 저장
5. Source Control에서 파일 stage ('+' 버튼)
6. Commit 메시지 입력 후 commit

### 7.8 Git History 보기 (GitLens 사용 시)

GitLens 확장을 설치했다면 더 많은 기능을 사용할 수 있습니다:

- 파일 위에 마우스를 올리면 최근 변경 정보 표시
- 각 줄 옆에 마지막 수정자와 시간 표시
- 좌측 사이드바에 GitLens 아이콘 추가 (commit 이력 탐색)
- 파일 우클릭 → 'Open File History' 등의 옵션 사용

---

## 8. 팀 프로젝트 워크플로우 제안

4명이 효율적으로 협업하기 위한 Git 워크플로우를 제안합니다.

### 8.1 기본 규칙

1. **main 브랜치는 항상 작동하는 코드만 유지**
   - 테스트되지 않은 코드는 절대 main에 직접 push하지 않기

2. **작업 시작 전 항상 최신 코드 받아오기**
   - `git pull origin main`으로 시작

3. **의미 있는 단위로 자주 commit**
   - 하루 종일 작업하고 한 번에 commit하지 않기

4. **명확한 commit 메시지 작성**
   - 팀원들이 이해할 수 있도록 구체적으로

### 8.2 간단한 워크플로우 (추천)

4명 정도의 작은 팀에서는 다음과 같이 단순하게 진행할 수 있습니다:

**매일 작업 시작:**
```bash
git pull origin main
# 작업 진행
```

**작업 중간중간:**
```bash
git add .
git commit -m "작업 내용"
```

**작업 완료 후:**
```bash
git pull origin main  # 최신 코드 확인
git push origin main
```

> 💡 **팁**: 이 방법은 간단하지만, 같은 파일을 동시에 수정하면 conflict가 발생할 수 있습니다. 팀원들과 작업 영역을 나누는 것이 중요합니다.

### 8.3 브랜치를 활용한 워크플로우 (권장)

조금 더 체계적으로 작업하고 싶다면 브랜치를 활용하세요:

**1. 새 기능 작업 시작:**
```bash
git checkout main
git pull origin main
git checkout -b feature/login
```

**2. 브랜치에서 작업:**
```bash
# 작업 진행
git add .
git commit -m "로그인 기능 구현"
git push origin feature/login
```

**3. 작업 완료 후 main에 병합:**

방법 A: 직접 병합 (간단한 경우)
```bash
git checkout main
git pull origin main
git merge feature/login
git push origin main
```

방법 B: Pull Request 사용 (추천, 부록 참조)
- GitHub에서 Pull Request를 만들어 팀원들의 리뷰 후 병합

**4. 브랜치 삭제:**
```bash
git branch -d feature/login
```

### 8.4 역할 분담 예시

팀원들과 역할을 명확히 나누면 conflict를 줄일 수 있습니다:

- **팀원 A**: 프론트엔드 (HTML, CSS)
- **팀원 B**: 백엔드 (서버, API)
- **팀원 C**: 데이터베이스
- **팀원 D**: 테스트 및 문서화

### 8.5 소통 규칙

- 매일 또는 주기적으로 팀 미팅 (진행 상황 공유)
- 같은 파일을 수정해야 할 때는 미리 공지
- 큰 변경사항은 팀원들에게 알리기
- 문제가 생기면 혼자 해결하려 하지 말고 팀원들과 상의

---

## 9. 자주 하는 실수와 해결법

### 9.1 실수 1: Commit하지 않고 Pull

**증상:**
```
error: Your local changes would be overwritten by merge
```

**해결법:**
```bash
git add .
git commit -m "작업 중인 내용 저장"
git pull origin main
```

### 9.2 실수 2: 잘못된 브랜치에 Commit

**증상:**
main 브랜치에서 작업하려 했는데 다른 브랜치에서 commit

**해결법 (아직 push하지 않았다면):**
```bash
git checkout main
git cherry-pick <commit-hash>
```

또는 그냥 브랜치를 병합하는 것이 더 쉬울 수 있습니다.

### 9.3 실수 3: Push한 Commit 취소하고 싶을 때

**증상:**
잘못된 코드를 push했는데 되돌리고 싶음

**해결법:**
```bash
git revert <commit-hash>  # 해당 commit을 취소하는 새 commit 생성
git push origin main
```

> ⚠️ **주의**: `git reset`은 이미 push한 commit에는 사용하지 마세요. 팀원들의 저장소와 충돌이 발생합니다.

### 9.4 실수 4: .gitignore 설정 잊음

**증상:**
불필요한 파일들(환경 설정, 빌드 파일 등)이 commit됨

**해결법:**

1. 프로젝트 루트에 `.gitignore` 파일 생성
2. 무시할 파일 패턴 작성:

```
node_modules/
*.pyc
.env
*.log
__pycache__/
.vscode/
*.swp
```

3. 이미 commit된 파일 제거:
```bash
git rm --cached <file>
git commit -m ".gitignore 적용"
```

> 💡 **팁**: GitHub에서 다양한 언어/프레임워크의 .gitignore 템플릿을 제공합니다. 검색해서 사용하세요!

### 9.5 도움 받기

문제가 발생했을 때 도움을 받을 수 있는 방법:

- `git status`를 실행하여 현재 상태 확인
- 에러 메시지를 정확히 읽고 이해하기
- 구글에서 에러 메시지로 검색
- 팀원들에게 도움 요청
- Stack Overflow 검색

---

## 부록: Pull Request(PR) 사용하기

Pull Request는 코드 리뷰와 협업을 위한 GitHub의 강력한 기능입니다. 작은 팀에서도 사용하면 코드 품질을 높이고 실수를 줄일 수 있습니다.

### A.1 Pull Request란?

Pull Request(PR)는 내가 작업한 브랜치의 변경사항을 main 브랜치에 병합해달라고 요청하는 것입니다. 이를 통해:

- 팀원들이 코드를 리뷰할 수 있음
- 병합 전에 문제를 발견하고 수정할 수 있음
- 변경 이력을 명확하게 관리할 수 있음
- 토론과 피드백을 통해 코드 품질 향상

### A.2 Pull Request 만들기

#### 1단계: 브랜치에서 작업 완료

```bash
git checkout -b feature/new-feature
# 작업 진행
git add .
git commit -m "새 기능 구현"
git push origin feature/new-feature
```

#### 2단계: GitHub에서 PR 생성

1. GitHub 저장소 페이지로 이동
2. 'Pull requests' 탭 클릭
3. 'New pull request' 버튼 클릭
4. base 브랜치: main, compare 브랜치: feature/new-feature 선택
5. 'Create pull request' 클릭

> 💡 **팁**: Push 후 GitHub 저장소 페이지를 새로고침하면 'Compare & pull request' 버튼이 나타나 더 쉽게 PR을 만들 수 있습니다.

#### 3단계: PR 정보 작성

PR을 생성할 때 다음 정보를 작성하세요:

- **제목**: 명확하고 간결하게 (예: '로그인 기능 구현')
- **설명**: 무엇을 변경했는지, 왜 변경했는지 설명

**설명 작성 예시:**
```markdown
## 변경 내용
- 로그인 UI 구현
- 로그인 API 연동
- 에러 처리 추가

## 테스트 방법
1. 로그인 페이지 접속
2. ID/PW 입력 후 로그인 버튼 클릭
3. 메인 페이지로 이동 확인
```

### A.3 코드 리뷰하기

팀원들은 PR을 검토하고 피드백을 남길 수 있습니다.

1. PR 페이지의 'Files changed' 탭 클릭
2. 변경된 코드 검토
3. 특정 줄에 댓글을 남기려면 줄 번호 옆 '+' 아이콘 클릭
4. 전체 리뷰 완료 후 'Review changes' 버튼 클릭
5. Approve (승인) 또는 Request changes (수정 요청) 선택

> 💡 **팁**: 피드백은 구체적이고 건설적으로 작성하세요. '이상해요' 대신 '이 부분을 함수로 분리하면 어떨까요?'처럼 제안하는 것이 좋습니다.

### A.4 PR 수정하기

리뷰를 받은 후 수정이 필요하다면:

1. 로컬에서 같은 브랜치에서 계속 작업
2. 수정 사항 commit
3. `git push origin feature/new-feature` (PR이 자동으로 업데이트됨)

### A.5 PR 병합하기

모든 리뷰가 완료되고 승인을 받았다면:

1. PR 페이지에서 'Merge pull request' 버튼 클릭
2. 병합 방법 선택:
   - **Merge commit**: 모든 commit 이력 유지 (기본값, 추천)
   - **Squash and merge**: 여러 commit을 하나로 합침
   - **Rebase and merge**: 선형 이력 유지
3. 'Confirm merge' 클릭
4. 'Delete branch' 버튼으로 브랜치 삭제 (선택사항)

> 💡 **팁**: 병합 후에는 로컬에서도 main 브랜치를 최신 상태로 업데이트하세요:
> ```bash
> git checkout main
> git pull origin main
> ```

### A.6 팀 프로젝트에서 PR 사용 규칙

PR을 효과적으로 사용하기 위한 규칙:

1. **작은 단위로 자주 PR 만들기**
   - 큰 PR은 리뷰하기 어렵습니다. 기능별로 나누어 PR을 만드세요

2. **최소 1명 이상의 승인 받기**
   - 혼자서 승인하고 병합하지 않기

3. **빠르게 리뷰하기**
   - PR이 올라오면 24시간 내에 리뷰하도록 노력

4. **테스트 후 병합하기**
   - 코드가 실제로 작동하는지 확인 후 병합

### A.7 PR vs 직접 병합

**PR을 사용해야 하는 경우:**
- 중요한 기능 추가나 수정
- 큰 변경사항
- 팀원들의 리뷰가 필요한 경우

**직접 병합해도 되는 경우:**
- 오타 수정
- 문서 업데이트
- 아주 작은 수정

---

## 마무리

이 가이드를 통해 Git과 GitHub의 기본적인 사용법을 익히셨기를 바랍니다. 처음에는 어렵고 복잡하게 느껴질 수 있지만, 계속 사용하다 보면 자연스럽게 익숙해질 것입니다.

### 기억해야 할 핵심 사항:

1. 작업 시작 전 항상 pull하기
2. 의미 있는 단위로 자주 commit하기
3. 명확한 commit 메시지 작성하기
4. 브랜치를 활용하여 안전하게 작업하기
5. 팀원들과 소통하며 협업하기

문제가 발생해도 당황하지 마세요. Git은 실수를 되돌릴 수 있는 다양한 방법을 제공합니다. 팀원들과 함께 문제를 해결하면서 더 많이 배울 수 있습니다.

**성공적인 팀 프로젝트를 응원합니다! 🎉**

---

### 추가 학습 자료:

- **Git 공식 문서**: https://git-scm.com/doc
- **GitHub Guides**: https://guides.github.com
- **생활코딩 Git 강좌**: https://opentutorials.org/course/2708
- **Learn Git Branching (인터랙티브)**: https://learngitbranching.js.org/?locale=ko
- **Pro Git Book (한글)**: https://git-scm.com/book/ko/v2