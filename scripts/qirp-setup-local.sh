#/bin/bash

# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear
# current dir

HOST_FILES=()

HOST_LIB_PATH="/usr/lib"
HOST_INCLUDE_PATH="/usr/include"
CONTAINER_LIB_PATH="/usr/local/lib"
CONTAINER_INCLUDE_PATH="/usr/local/include"

# libs and head flies of QNN SDK
HOST_FILES+=("-v /usr/bin/qtld-net-run:/usr/bin/qtld-net-run")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libcalculator.so:${CONTAINER_LIB_PATH}/libcalculator.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libhta_hexagon_runtime_snpe.so:${CONTAINER_LIB_PATH}/libhta_hexagon_runtime_snpe.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libPlatformValidatorShared.so:${CONTAINER_LIB_PATH}/libPlatformValidatorShared.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnChrometraceProfilingReader.so:${CONTAINER_LIB_PATH}/libQnnChrometraceProfilingReader.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnCpu.so:${CONTAINER_LIB_PATH}/libQnnCpu.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnDsp.so:${CONTAINER_LIB_PATH}/libQnnDsp.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnDspNetRunExtensions.so:${CONTAINER_LIB_PATH}/libQnnDspNetRunExtensions.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnDspV66CalculatorStub.so:${CONTAINER_LIB_PATH}/libQnnDspV66CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnDspV66Stub.so:${CONTAINER_LIB_PATH}/libQnnDspV66Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnGenAiTransformer.so:${CONTAINER_LIB_PATH}/libQnnGenAiTransformer.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnGenAiTransformerCpuOpPkg.so:${CONTAINER_LIB_PATH}/libQnnGenAiTransformerCpuOpPkg.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnGenAiTransformerModel.so:${CONTAINER_LIB_PATH}/libQnnGenAiTransformerModel.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnGpu.so:${CONTAINER_LIB_PATH}/libQnnGpu.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnGpuNetRunExtensions.so:${CONTAINER_LIB_PATH}/libQnnGpuNetRunExtensions.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnGpuProfilingReader.so:${CONTAINER_LIB_PATH}/libQnnGpuProfilingReader.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtp.so:${CONTAINER_LIB_PATH}/libQnnHtp.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpNetRunExtensions.so:${CONTAINER_LIB_PATH}/libQnnHtpNetRunExtensions.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpOptraceProfilingReader.so:${CONTAINER_LIB_PATH}/libQnnHtpOptraceProfilingReader.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpPrepare.so:${CONTAINER_LIB_PATH}/libQnnHtpPrepare.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpProfilingReader.so:${CONTAINER_LIB_PATH}/libQnnHtpProfilingReader.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV68CalculatorStub.so:${CONTAINER_LIB_PATH}/libQnnHtpV68CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV68Stub.so:${CONTAINER_LIB_PATH}/libQnnHtpV68Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV69CalculatorStub.so:${CONTAINER_LIB_PATH}/libQnnHtpV69CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV69Stub.so:${CONTAINER_LIB_PATH}/libQnnHtpV69Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV73CalculatorStub.so:${CONTAINER_LIB_PATH}/libQnnHtpV73CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV73Stub.so:${CONTAINER_LIB_PATH}/libQnnHtpV73Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnHtpV75Stub.so:${CONTAINER_LIB_PATH}/libQnnHtpV75Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnJsonProfilingReader.so:${CONTAINER_LIB_PATH}/libQnnJsonProfilingReader.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnSaver.so:${CONTAINER_LIB_PATH}/libQnnSaver.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnSystem.so:${CONTAINER_LIB_PATH}/libQnnSystem.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libQnnTFLiteDelegate.so:${CONTAINER_LIB_PATH}/libQnnTFLiteDelegate.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSNPE.so:${CONTAINER_LIB_PATH}/libSNPE.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeDspV66Stub.so:${CONTAINER_LIB_PATH}/libSnpeDspV66Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHta.so:${CONTAINER_LIB_PATH}/libSnpeHta.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpPrepare.so:${CONTAINER_LIB_PATH}/libSnpeHtpPrepare.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpV68CalculatorStub.so:${CONTAINER_LIB_PATH}/libSnpeHtpV68CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpV68Stub.so:${CONTAINER_LIB_PATH}/libSnpeHtpV68Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpV73CalculatorStub.so:${CONTAINER_LIB_PATH}/libSnpeHtpV73CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpV73Stub.so:${CONTAINER_LIB_PATH}/libSnpeHtpV73Stub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpV75CalculatorStub.so:${CONTAINER_LIB_PATH}/libSnpeHtpV75CalculatorStub.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libSnpeHtpV75Stub.so:${CONTAINER_LIB_PATH}/libSnpeHtpV75Stub.so")

# libs for QNN TFLite GPU delegate
HOST_FILES+=("-v ${HOST_LIB_PATH}/libadreno_utils.so:${CONTAINER_LIB_PATH}/libadreno_utils.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libCB.so:${CONTAINER_LIB_PATH}/libCB.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libOpenCL.so:${CONTAINER_LIB_PATH}/libOpenCL.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libOpenCL_adreno.so:${CONTAINER_LIB_PATH}/libOpenCL_adreno.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libdmabufheap.so.0:${CONTAINER_LIB_PATH}/libdmabufheap.so.0")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libdmabufheap.so.0.0.0:${CONTAINER_LIB_PATH}/libdmabufheap.so.0.0.0")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libgsl.so:${CONTAINER_LIB_PATH}/libgsl.so")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libpropertyvault.so.0:${CONTAINER_LIB_PATH}/libpropertyvault.so.0")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libpropertyvault.so.0.0.0:${CONTAINER_LIB_PATH}/libpropertyvault.so.0.0.0")
HOST_FILES+=("-v ${HOST_LIB_PATH}/libllvm-qcom.so:${CONTAINER_LIB_PATH}/libllvm-qcom.so")

HOST_devices+=("--device=/dev/dri/card0")
HOST_devices+=("--device=/dev/dri/renderD128")
HOST_devices+=("--device=/dev/kgsl-3d0")
HOST_devices+=("--device=/dev/dma_heap/system")
HOST_devices+=("--device=/dev/dma_heap/qcom,system")
HOST_devices+=("--device=/dev/fastrpc-cdsp")
HOST_devices+=("--device=/dev/v4l-subdev8")
HOST_devices+=("--device=/dev/video0")
HOST_devices+=("--device=/dev/video33")
if [ -e /dev/video2 ]; then
    HOST_devices+=("--device=/dev/video2")
fi

# Define Docker image and container names
IMAGE_NAME="qirp-docker"
CONTAINER_NAME="qirp-samples-container"

ai_model_list=( \
    MediaPipeHandDetector.tflite,https://huggingface.co/qualcomm/MediaPipe-Hand-Detection/resolve/0d0b12ccd12b96457185ff9cfab75eb7c7ab3ad6/MediaPipeHandDetector.tflite?download=true  \
    MediaPipeHandLandmarkDetector.tflite,https://huggingface.co/qualcomm/MediaPipe-Hand-Detection/resolve/0d0b12ccd12b96457185ff9cfab75eb7c7ab3ad6/MediaPipeHandLandmarkDetector.tflite?download=true  \
    anchors_palm.npy,https://raw.githubusercontent.com/zmurez/MediaPipePyTorch/65f2549ba35cd61dfd29f402f6c21882a32fabb1/anchors_palm.npy  \
    ResNet101Quantized.tflite,https://huggingface.co/qualcomm/ResNet101Quantized/resolve/653916aac7c732d28863aa449176299ba2890c15/ResNet101Quantized.tflite?download=true  \
    imagenet_labels.txt,https://raw.githubusercontent.com/quic/ai-hub-models/refs/heads/main/qai_hub_models/labels/imagenet_labels.txt  \
 )

#--------setup in qclinux -----------
function docker_check_install_depends(){
    # Check if Docker image exists
    if ! docker images --format "{{.Repository}}" | grep -w ^$IMAGE_NAME$ ;then
        echo "Docker image $IMAGE_NAME not found, loading from $IMAGE_PATH ..."
        if [ -f $IMAGE_PATH ];then
            docker load -i "$IMAGE_PATH"
            if [ $? -eq 0 ]; then
                echo "Docker image successfully loaded."
            else
                echo "Error loading Docker image."
                return  1
            fi
        else
            echo "no docker image $IMAGE_NAME in $IMAGE_PATH"
            #docker pull --platform=linux/arm64/v8 ros:$ROS_DISTRO
        fi
    else
        echo "Docker image $IMAGE_NAME already exists."
    fi

    # Check if Docker container exists
    if ! docker ps -a --format "{{.Names}}" | grep -w ^$CONTAINER_NAME$; then
        echo "Docker container $CONTAINER_NAME not found, starting..."
        docker run -d -it --rm \
            -e LOCAL_USER_NAME=$(whoami) \
            -e LOCAL_USER_ID=$(id | awk -F "(" '{print $1}' | sed 's/uid=//') \
            -e LOCAL_GROUP_ID=$(id | awk -F "(" '{print $2}' | awk -F " " '{print $NF}' | sed 's/gid=//') \
            -v $Linux_DIR:$Linux_DIR \
            ${HOST_FILES[@]} \
            ${HOST_devices[@]} \
            --network=host \
            --name=$CONTAINER_NAME \
            --security-opt seccomp=unconfined \
            $IMAGE_NAME:latest
        if [ $? -eq 0 ]; then
            echo "Docker container successfully started."
        else
            echo "Error starting Docker container."
            return  1
        fi

    else
        echo "Docker container $CONTAINER_NAME already exists."
    fi

    docker exec -it -u root $CONTAINER_NAME /bin/bash
    if [ $? -eq 0 ]; then
        echo "Docker container successfully started."
    else
        echo "Error starting Docker container."
        return  1
    fi
}
function linux_env_setup(){
    echo "Linux setup"

    SDK_NAME="QIRP_SDK"

    # Install colcon and build dependencies for local development
    echo "Installing colcon and build dependencies..."
    
    # Check if build tools are available (they're already installed on QIRP)
    echo "Verifying build tools..."
    for tool in cmake make python3; do
        if command -v $tool &> /dev/null; then
            echo "✓ $tool is available"
        else
            echo "✗ $tool not found"
        fi
    done
    
    # Check for QIRP-specific compiler
    if command -v aarch64-qcom-linux-gcc &> /dev/null; then
        echo "✓ aarch64-qcom-linux-gcc is available"
    else
        echo "✗ aarch64-qcom-linux-gcc not found"
    fi
    
    # Check for git (not available via opkg on QIRP)
    if command -v git &> /dev/null; then
        echo "✓ git is available"
    else
        echo "⚠ git not found (not available via opkg on QIRP)"
        echo "  GitPython installed for Python git operations"
        echo "  For full git functionality, consider using a different approach"
    fi
    
    # Install pip if not available
    if ! command -v pip3 &> /dev/null; then
        echo "Installing pip..."
        python3 -m ensurepip --upgrade
    fi
    
    # Install colcon using pip
    if ! command -v colcon &> /dev/null; then
        echo "Installing colcon..."
        python3 -m pip install colcon-common-extensions --break-system-packages
    else
        echo "✓ colcon is already installed"
    fi
    
    # Install scientific Python packages if not available
    echo "Checking scientific packages..."
    python3 -c "import numpy" 2>/dev/null && echo "✓ numpy is available" || {
        echo "Installing numpy..."
        python3 -m pip install numpy --break-system-packages
    }
    
    python3 -c "import scipy" 2>/dev/null && echo "✓ scipy is available" || {
        echo "Installing scipy..."
        python3 -m pip install scipy --break-system-packages
    }
    
    # Install GitPython for git operations (since git CLI not available via opkg)
    python3 -c "import git" 2>/dev/null && echo "✓ GitPython is available" || {
        echo "Installing GitPython..."
        python3 -m pip install GitPython --break-system-packages
    }

    #common environment variables export
    export PATH=/bin/aarch64-oe-linux-gcc11.2:/usr/bin:/usr/bin/qim/:${PATH}
    export LD_LIBRARY_PATH=/lib/aarch64-oe-linux-gcc11.2:/usr/lib:/usr/lib/qim:/lib:${LD_LIBRARY_PATH}

    #ROS environment variables export
    export AMENT_PREFIX_PATH=/usr:${AMENT_PREFIX_PATH}
    export PYTHONPATH=/usr/lib/python3.10/site-packages:${PYTHONPATH}

    #gst environment variables export
    export GST_PLUGIN_PATH=/usr/lib/qim/gstreamer-1.0:/usr/lib/gstreamer-1.0:${GST_PLUGIN_PATH}
    export GST_PLUGIN_SCANNER=/usr/libexec/qim/gstreamer-1.0/gst-plugin-scanner:/usr/libexec/gstreamer-1.0/gst-plugin-scanner:${GST_PLUGIN_SCANNER}

    #qnn environment variables export
    export ADSP_LIBRARY_PATH=/usr/lib/rfsa/adsp:${ADSP_LIBRARY_PATH}
    export HOME=/opt
    source /usr/bin/ros_setup.sh
}

function check_network_connection() {
    local wifi_connected=0
    local ethernet_connected=0

    # Check WiFi
    local wifi_status=$(nmcli -t -f WIFI g)
    if [[ "$wifi_status" == "enabled" ]]; then
        local connection_status=$(nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes' | cut -d':' -f2)
        if [[ -n "$connection_status" ]]; then
            echo "Connected to WiFi: $connection_status"
            wifi_connected=1
        else
            echo "Not connected to any WiFi network."
        fi
    else
        echo "WiFi is disabled."
    fi

    # Check Ethernet
    local ethernet_status=$(nmcli -t -f DEVICE,STATE dev | grep '^eth' | awk -F: '{print $2}')
    if [[ "$ethernet_status" == "connected" ]]; then
        echo "Ethernet is connected."
        ethernet_connected=1
    else
        echo "Ethernet is not connected."
    fi

    # Return 0 if either WiFi or Ethernet is connected
    if [[ $wifi_connected -eq 1 || $ethernet_connected -eq 1 ]]; then
        return 0
    else
        return 1
    fi
}

function download_ai_model(){
    if [ ! -d /opt/model/ ];then
        echo "no model direction in /opt/model"
        mkdir /opt/model
    fi

    for model in "${ai_model_list[@]}"; do
        # using IFS parse name and link
        IFS=',' read -r name link <<< "$model"
        if [ -f /opt/model/$name ];then
            echo "/opt/model/$name has download in device"
        else
            wget -O /opt/model/$name $link
            if [ $? -eq 0 ]; then
                echo "echo Downloading $name from $link  successfully "
            else
                echo "Downloading $name from $link  fail"
            fi
        fi
    done
}

function show_help() {
    echo "Usage: source /root/ros2_ws/scripts/qirp-setup-local.sh [OPTION]"
    echo ""
    echo "Options:"
    echo "  -h, --help        Show this help message."
    echo "  -m, --model       Download AI sample models required for execution."
    echo "  -d, --docker      Load Docker on the device."
    echo "  -l, --local       Setup for local development with colcon and build tools."
    echo "  --docker_path     Specify the local Docker image path (default: /home/qirp-docker.tar.gz)."
    echo ""
    echo "Examples:"
    echo "  source /root/ros2_ws/scripts/qirp-setup-local.sh --help"
    echo "  source /root/ros2_ws/scripts/qirp-setup-local.sh --local"
    echo "  source /root/ros2_ws/scripts/qirp-setup-local.sh --model"
    echo "  source /root/ros2_ws/scripts/qirp-setup-local.sh --docker --docker_path /your/custom/path.tar.gz"
}


#--------------main point--------------#
main(){
    
    case "$1" in
        -h|--help)
            show_help
            return 1
            ;;
        -l|--local)
            echo "Setting up QIRP QCLinux for local development with colcon"
            linux_env_setup
            echo "Local development setup completed successfully!"
            echo "You can now use colcon to build ROS2 packages locally."
            echo "Example: colcon build --packages-select your_package_name"
            return 0
            ;;
        -m|--model)
            check_network_connection
            if [[ $? -eq 0 ]]; then
                echo "Network checks passed successfully!"
                download_ai_model
            else
                echo "Something went wrong. Please check network status."
                return 1
            fi
            ;;
        -d|--docker)
            echo "building docker image..."
            date -s "2025-03-20"
            if [ "$2" == "--docker_path" ]; then
                echo "loading docker path from $3"
                IMAGE_PATH=$3
            else
                IMAGE_PATH="/home/qirp-docker.tar.gz"
            fi
            docker_check_install_depends
            if [[ $? -eq 0 ]]; then
                echo "docker load successfully!"
            else
                echo "docker load  wrong."
                return 1
            fi
            ;;
        *)
            echo "Setting up QIRP QCLinux for execution on device"
        ;;
    esac
    linux_env_setup
    echo "Setting up QIRP QCLinux successfully"
}

main $1 $2 $3
