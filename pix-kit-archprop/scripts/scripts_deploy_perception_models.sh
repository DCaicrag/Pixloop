#!/bin/bash

DIRPATH_ROOT=".."
DIRPATH_PERC_MODELS="$DIRPATH_ROOT""/src/pix-citybot-v2-perception-models"
DIRPATH_PERC_SRC="$DIRPATH_ROOT""/src/autoware/AutowareArchitectureProposal.iv/perception"

function info() {
    >&2 echo -e "[ INFO ] ${1}"
}

function error() {
    local -r ERROR_TEXT="\033[0;31m"  # red
    local -r NO_COLOR="\033[0m"
    >&2 echo -e "${ERROR_TEXT}[ ERROR ] ${1}${NO_COLOR}"
}

function hint() {
    local -r ERROR_TEXT="\033[0;36m"  # red
    local -r NO_COLOR="\033[0m"
    echo -e "${ERROR_TEXT}Hint: ${1}${NO_COLOR}"
}

function check_dirpath() {
    if [ ! -d "$1" ]; then
        error "Directory not found. (dirpath: \"$1\")"
        hint "Please check the current branch of the repository and if submodules have been initialized properly."
        exit 1
    fi
}

function check_filepath() {
    if [ ! -f "$1" ]; then
        error "File not found. (filepath: \"$1\")"
        hint "Please check the current branch of the repository and if submodules have been initialized properly."
        exit 1
    fi
}

function deploy_model() {
    local -r MODEL_NAME="$1"
    local -r FILEPATH_MODEL_SRC="$2"
    local -r FILEPATH_MODEL_DST="$3"
    local -r FILENAME_MODEL_DST=`basename "$FILEPATH_MODEL_DST"`
    local -r DIRPATH_MODEL_DST=${FILEPATH_MODEL_DST%"/$FILENAME_MODEL_DST"}

    info "Deploying model \"$MODEL_NAME\". (src: \"$FILEPATH_MODEL_SRC\", dest: \"$FILEPATH_MODEL_DST\")"

    check_filepath "$FILEPATH_MODEL_SRC"
    check_dirpath "$DIRPATH_MODEL_DST"

    if [ -f "$FILEPATH_MODEL_DST" ]; then
        info "Overwriting original data file for model \"$MODEL_NAME\"."
    fi

    cp "$FILEPATH_MODEL_SRC" "$FILEPATH_MODEL_DST"

    info "Successfully deployed model \"$MODEL_NAME\"."
}

function main() {
    check_dirpath "$DIRPATH_PERC_MODELS"
    check_dirpath "$DIRPATH_PERC_SRC"

    deploy_model "ros_yolo_best" \
                 "$DIRPATH_PERC_MODELS""/ros_yolo/best.pt" \
                 "$DIRPATH_PERC_SRC""/object_recognition/detection/tensorrt_yolo/model/best.pt"

    deploy_model "ros_yolo_yolov5m" \
                 "$DIRPATH_PERC_MODELS""/ros_yolo/yolov5m.pt" \
                 "$DIRPATH_PERC_SRC""/object_recognition/detection/tensorrt_yolo/model/yolov5m.pt"

    deploy_model "trt_yolo_yolov5m" \
                 "$DIRPATH_PERC_MODELS""/trt_yolo/yolov5m.onnx" \
                 "$DIRPATH_PERC_SRC""/object_recognition/detection/trt_yolo/model/yolov5m.onnx"
}

main "$@"
