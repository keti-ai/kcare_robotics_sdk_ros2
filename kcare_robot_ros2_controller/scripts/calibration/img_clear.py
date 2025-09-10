import os
import glob

def clear_images(directory):
    # 이미지 파일 확장자 목록
    image_extensions = ['*.png', '*.jpg', '*.jpeg','*.json']

    # 모든 이미지 파일을 찾기
    for ext in image_extensions:
        # 주어진 디렉토리에서 해당 확장자를 가진 파일을 찾기
        image_files = glob.glob(os.path.join(directory, ext))

        # 파일이 존재하면 삭제
        for image_file in image_files:
            try:
                os.remove(image_file)
                print(f"삭제된 이미지: {image_file}")
            except Exception as e:
                print(f"파일 삭제 중 오류 발생: {e}")

def main():
    # 이미지 파일이 저장된 폴더 경로
    script_dir = os.path.dirname(os.path.realpath(__file__))  # 현재 스크립트 위치
    logs_dir = os.path.join(script_dir, '..', 'logs')  # logs 폴더 경로

    # 폴더가 존재하는지 확인
    if os.path.exists(logs_dir):
        clear_images(logs_dir)
    else:
        print(f"경로가 존재하지 않습니다: {logs_dir}")

if __name__ == "__main__":
    main()
