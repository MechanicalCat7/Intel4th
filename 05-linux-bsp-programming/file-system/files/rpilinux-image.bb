require recipes-core/images/core-image-minimal-dev.bb 

IMAGE_INSTALL += "libstdc++ mtd-utils" 
IMAGE_INSTALL += "openssh openssl openssh-sftp-server " 
IMAGE_INSTALL += "python3" 
IMAGE_INSTALL += "gcc libgcc glibc" 
