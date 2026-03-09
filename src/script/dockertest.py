import subprocess
compose = ["sudo", "docker", "compose", "up"]
subprocess.run(compose,
               cwd="/home/mscp/capstone/src",
               check=True)