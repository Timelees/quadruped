"""Test-only strip masking for the Gazebo causal map-reveal protocol.

The wrong-floor condition deliberately violates physical map conservatism.
It is a stress intervention, never a source of deployment certificates.
"""
import math
import copy
import numpy as np


def set_update_release(message, released):
    """Release the method with the same immutable map used for intervention."""
    layer='swing_update_release_test_only'
    if layer not in message.layers:
        message.layers.append(layer)
        message.data.append(copy.deepcopy(message.data[0]))
    data=message.data[message.layers.index(layer)]
    data.data=[float(released)]*len(data.data)


def swing_spans_bar(liftoff_x, forecast_x, centre, width, foot_radius=.020):
    """Require free footholds on both sides of this static box, including the foot."""
    return bool(forecast_x) and liftoff_x < centre-width/2-foot_radius and max(forecast_x) > centre+width/2+foot_radius


def mask_strip(message, rectangle, unknown=False):
    """Modify only the requested world rectangle, including rolling buffers."""
    info, resolution = message.info, message.info.resolution
    nx, ny = round(info.length_x/resolution), round(info.length_y/resolution)
    xmin, xmax, ymin, ymax = rectangle
    x=info.pose.position.x+info.length_x/2-(np.arange(nx)+.5)*resolution
    y=info.pose.position.y+info.length_y/2-(np.arange(ny)+.5)*resolution
    mask=((xmin<=x)&(x<=xmax))[:,None] & ((ymin<=y)&(y<=ymax))[None,:]
    mask=np.roll(mask,(message.outer_start_index,message.inner_start_index),(0,1))
    changed = 0
    for layer, data in zip(message.layers, message.data):
        if layer not in ('elevation', 'elevation_certified', 'smooth_planar'):
            continue
        column_major = data.layout.dim[0].label == 'column_index'
        values=np.asarray(data.data).copy()
        values[mask.ravel(order='F' if column_major else 'C')]=(
            math.nan if unknown and layer=='elevation_certified' else 0.)
        data.data=values.tolist()
        changed+=int(mask.sum())
    return changed


def known_course_layer(message, bars=None):
    """Exact static test lane heights for a labelled controller mechanism test.

    This oracle is never enabled in ordinary runs or perception calibration.
    Only elevation_certified changes; the perception cost layers and planar
    foothold regions are retained, so this is a certificate-evidence intervention.
    """
    if 'elevation_certified' not in message.layers:
        raise ValueError('Controlled truth reveal requires the upstream certificate layer')
    data = message.data[message.layers.index('elevation_certified')]
    info, r = message.info, message.info.resolution
    nx,ny = round(info.length_x/r),round(info.length_y/r)
    column_major = data.layout.dim[0].label == 'column_index'
    if bars is None:
        bars = [(1.1,.06,.04),(1.8,.08,.07),(2.5,.12,.05),(3.2,.06,.09)]
    x=info.pose.position.x+info.length_x/2-(np.arange(nx)+.5)*r
    y=info.pose.position.y+info.length_y/2-(np.arange(ny)+.5)*r
    lane=((-0.6<=x)&(x<=5.))[:,None] & ((-3.8<=y)&(y<=-2.2))[None,:]
    heights=np.zeros((nx,ny))
    for bar in bars:
        centre,width,height=bar[:3]
        centre_y,extent_y=bar[3:] if len(bar)==5 else (-3.,1.4)
        occupied=(np.abs(x-centre)<=width/2+r/2)[:,None] & (np.abs(y-centre_y)<=extent_y/2+r/2)[None,:]
        heights[occupied]=np.maximum(heights[occupied],height)
    order='F' if column_major else 'C'
    shift=(message.outer_start_index,message.inner_start_index)
    mask=np.roll(lane,shift,(0,1)).ravel(order=order)
    values=np.asarray(data.data).copy()
    values[mask]=np.roll(heights,shift,(0,1)).ravel(order=order)[mask]
    data.data=values.tolist()
