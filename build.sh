if [ $# -eq 0 ]; then
    echo "请提供作业号。"
    exit 1
fi

HW=$1

cmake --preset hw$HW
cmake --build build/hw$HW