FROM ubuntu:22.04

# 기본 패키지 설치
RUN apt update && apt install -y \
    python3 \
    python3-pip \
    git \
    curl \
    build-essential \
    libgl1-mesa-glx \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 설정
WORKDIR /app

# 파이썬 패키지 설치
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# 전체 코드 복사
COPY . .

# 기본 진입점
CMD ["/bin/bash"]
