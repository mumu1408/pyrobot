echo "test..."

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)

PROJECT_FOLDER=$(dirname $(dirname $(dirname $(dirname $(dirname $script_dir)))))

echo $script_dir
echo $PROJECT_FOLDER