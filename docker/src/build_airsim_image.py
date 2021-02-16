import argparse
import subprocess

def main():
    parser = argparse.ArgumentParser(description='FSDS simulator docker image builder')
    parser.add_argument('--target_image', type=str, help='base image name AND tag')

    args = parser.parse_args()
    build_docker_image(args)

def build_docker_image(args):
    dockerfile = 'Dockerfile_binary'
    if not args.target_image:
        args.target_image = 'fsdsairsim_binary' + ':vulkan-ubuntu18.04'

    docker_command = ['docker', 'build', '--network=host', \
                        '-t', args.target_image, \
                        '-f',  dockerfile, \
                        '--build-arg', 'BASE_IMAGE=nvidia/vulkan:1.1.121-cuda-10.1-beta.1-ubuntu18.04', \
                        '.']
    print(" ".join(docker_command))
    subprocess.call(docker_command)

if __name__=="__main__":
    main()
