import subprocess
class volumecontrol():
    def controlvol(self,vol):
        vol_str = f"{int(vol)}%"
        subprocess.run(["amixer","set","Master",vol_str])
