"""GStreamer pipeline strings and decoder order."""

from __future__ import annotations

from pashupati_bridge.core.rtsp import decoder_candidates, gstreamer_pipeline, opencv_has_gstreamer
import pytest

URL = 'rtsp://192.168.1.120:8554/test'


def test_jetson_pipeline_uses_the_hardware_decoder():
    """The Jetson pipeline decodes with nvv4l2decoder and hands BGR frames to the appsink."""
    pipeline = gstreamer_pipeline(URL, 'h264', 'jetson', latency_ms=50)
    assert pipeline.startswith(f'rtspsrc location={URL} latency=50 protocols=tcp')
    assert 'rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv' in pipeline
    assert pipeline.endswith('video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false')


def test_rate_and_size_are_applied_inside_the_pipeline():
    """Frames are dropped and scaled before the colour conversion, on the Jetson in hardware."""
    jetson = gstreamer_pipeline(URL, 'h264', 'jetson', max_fps=5.0, width=1280, height=720)
    assert (
        'nvvidconv ! video/x-raw,format=BGRx,width=1280,height=720 ! '
        'videorate max-rate=5 ! videoconvert' in jetson
    )
    software = gstreamer_pipeline(URL, 'h264', 'software', max_fps=2.5, width=640, height=360)
    assert (
        'avdec_h264 ! videorate max-rate=3 ! '
        'videoscale ! video/x-raw,width=640,height=360 ! videoconvert' in software
    )
    assert 'videorate' not in gstreamer_pipeline(URL, 'h264', 'jetson', max_fps=0.0)
    assert 'width' not in gstreamer_pipeline(URL, 'h264', 'software', width=1280, height=0)


def test_software_pipeline_and_h265():
    """The software pipeline uses libav; h265 changes the depayloader too."""
    pipeline = gstreamer_pipeline(URL, 'H265', 'software')
    assert 'rtph265depay ! h265parse ! avdec_h265 ! videoconvert' in pipeline
    with pytest.raises(ValueError):
        gstreamer_pipeline(URL, 'mjpeg', 'software')
    with pytest.raises(ValueError):
        gstreamer_pipeline(URL, 'h264', 'ffmpeg')


def test_decoder_order():
    """The auto mode tries the Jetson first and FFmpeg last; an explicit choice stays alone."""
    assert decoder_candidates('auto') == ['jetson', 'software', 'ffmpeg']
    assert decoder_candidates('software') == ['software']
    with pytest.raises(ValueError):
        decoder_candidates('cuda')


def test_opencv_gstreamer_detection():
    """The build information tells whether OpenCV can run a GStreamer pipeline."""
    assert opencv_has_gstreamer('  Video I/O:\n    GStreamer:                   YES (1.20.3)\n')
    assert not opencv_has_gstreamer('    GStreamer:                   NO\n')
    assert not opencv_has_gstreamer('')
