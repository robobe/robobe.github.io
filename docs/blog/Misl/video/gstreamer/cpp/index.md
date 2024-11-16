---
tags:
    - gstreamer
    - cpp
    - 
---

# Gstreamer cpp 
Build and manipulate gstreamer pipe using cpp

## Install

```bash
sudo apt install g++ cmake pkg-config
#
sudo apt-get install libgstreamer1.0-0 \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav \
gstreamer1.0-tools \
gstreamer1.0-libav
```


```bash
sudo apt install libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-good1.0-dev \
  libgstreamer-plugins-bad1.0-dev 

```

---

## Simple Demo
```cpp
#include <iostream>
#include <gst/gst.h>

int main(int arg, char *argv[]) {
    GstElement *pipeline = nullptr;
    GstBus *bus = nullptr;
    GstMessage *msg = nullptr;

    // gstreamer initialization
    gst_init(&arg, &argv);

    
    // building pipeline
    //"playbin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm",
    pipeline = gst_parse_launch(
            "videotestsrc name=src ! videoconvert ! autovideosink",
            nullptr);

    
    // GstElement* element = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    // g_object_set(G_OBJECT(element), "pattern", 1, NULL);


    // start playing
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    //wait until error or EOS ( End Of Stream )
    bus = gst_element_get_bus(pipeline);
    msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                     static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    // free memory
    if (msg != nullptr)
        gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}

```

```bash
cmake_minimum_required(VERSION 3.16)

project(gstreamer_01) #Project name

set(CMAKE_CXX_STANDARD 14) #setting C++ 14 standard
find_package(PkgConfig) #finding pkg-config is a helper tool
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

#including GStreamer header files directory
include_directories(
        ${GLIB_INCLUDE_DIRS}
        ${GSTREAMER_INCLUDE_DIRS}
)

#linking GStreamer library directory
link_directories(
        ${GLIB_LIBRARY_DIRS}
        ${GSTREAMER_LIBRARY_DIRS}
)

#building target executable
add_executable(${PROJECT_NAME} src/main.cpp)

#linking Gstreamer library with target executable
target_link_libraries(${PROJECT_NAME} ${GSTREAMER_LIBRARIES})
```

---

## Demo2: using caps
In GStreamer, caps (short for capabilities) describe the type of data a GStreamer element can produce, accept, or transform. They define the format, properties, and restrictions of the media stream at a specific point in a GStreamer pipeline. Caps ensure that connected elements in a pipeline agree on the format of the data they handle.

```cpp
#include <gst/gst.h>
#include <iostream>

int main(int argc, char *argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Create the pipeline
    GstElement *pipeline = gst_pipeline_new("test-pipeline");

    // Create elements
    GstElement *source = gst_element_factory_make("videotestsrc", "source");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "filter");
    GstElement *sink = gst_element_factory_make("autovideosink", "sink");

    if (!pipeline || !source || !capsfilter || !sink) {
        std::cerr << "Failed to create GStreamer elements" << std::endl;
        return -1;
    }

    // Set properties directly using g_object_set
    g_object_set(source,
                 "pattern", 1,               // Pattern type (0 is default)
                 "num-buffers", 100,         // Limit number of buffers (optional)
                 nullptr);

    // Create and set caps
    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",       // Media type
        "width", G_TYPE_INT, 320,  // Set resolution width
        "height", G_TYPE_INT, 240, // Set resolution height
        "framerate", GST_TYPE_FRACTION, 30, 1,  // Set framerate
        nullptr);

    g_object_set(capsfilter, "caps", caps, nullptr);
    gst_caps_unref(caps); // Free the caps when done

    // Build the pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, capsfilter, sink, nullptr);
    if (!gst_element_link_many(source, capsfilter, sink, nullptr)) {
        std::cerr << "Failed to link elements" << std::endl;
        return -1;
    }

    // Run the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to set pipeline to PLAYING state" << std::endl;
        return -1;
    }

    // Wait until error or EOS
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus, GST_CLOCK_TIME_NONE, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    // Handle messages
    if (msg != nullptr) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug_info);
                std::cerr << "Error received from element " << GST_OBJECT_NAME(msg->src)
                          << ": " << err->message << std::endl;
                g_clear_error(&err);
                g_free(debug_info);
                break;
            case GST_MESSAGE_EOS:
                std::cout << "End-Of-Stream reached." << std::endl;
                break;
            default:
                std::cerr << "Unexpected message received." << std::endl;
                break;
        }
        gst_message_unref(msg);
    }

    // Clean up
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}

```

---

## reference
- [Creating Gstreamer Multimedia Pipeline With C++ Part 1](https://medium.com/analytics-vidhya/creating-gstreamer-multimedia-pipeline-with-c-part-1-a7f0f86b5e1f)