__author__ = 'suchet'


import sys
import datetime
import numpy as np
import Pysolar


def logStrToTime(strTime):
    """
    Convert log time string to np.datetime format
    """
    t_str = '-'.join([strTime[0:4],strTime[4:6], strTime[6:9]])
    t_str += ':'.join((strTime[9:11], strTime[11:13], strTime[13:])) + 'Z'
    return np.datetime64(np.datetime64(t_str).astype(datetime.datetime))

def lat_long_to_elaz(latitude, longitude, utc_datetime):
    """
    Obtain the sun position using latitude, longitude and time
    :param latitude: in degrees
    :param longitude: in degrees
    :param utc_datetime: in datetime.datetime format
    :return: azimuth and elevation of the sun

    Notes:
    ------------
    Azimuth and elevation is calculated according to the Pysolar library.
    Documentation: http://docs.pysolar.org/en/latest/
    The authors measure positive azimuth as estimates east of south. This is converted instead to positive estimates
    measured EAST from NORTH instead.
    """

    # Calculate azimuth
    south_azimuth = Pysolar.GetAzimuth(latitude, longitude, utc_datetime=utc_datetime)
    # Reorient azimuth so that positive measures East from North
    azimuth = 180 - (south_azimuth+360) % 360

    # Calculate elevation
    elevation = Pysolar.GetAltitude(latitude, longitude, utc_datetime=utc_datetime)

    return elevation, azimuth

def spherical_to_cartesian(spherical_vect):
    """Convert the spherical coordinate vector [r, theta, phi] to the Cartesian vector [x, y, z].

    The parameter r is the radial distance, theta is the polar angle, and phi is the azimuth.


    @param spherical_vect:  The spherical coordinate vector [r, theta, phi].
    @type spherical_vect:   3D array or list
    @param cart_vect:       The Cartesian vector [x, y, z].
    @type cart_vect:        3D array or list
    """

    # Trig alias.
    sin_theta = np.sin(spherical_vect[:, 1])

    # The vector.
    cart_vect = np.ones(spherical_vect.shape)
    cart_vect[:, 0] = spherical_vect[:, 0] * np.cos(spherical_vect[:, 2]) * sin_theta
    cart_vect[:, 1] = spherical_vect[:, 0] * np.sin(spherical_vect[:, 2]) * sin_theta
    cart_vect[:, 2] = spherical_vect[:, 0] * np.cos(spherical_vect[:, 1])

    return cart_vect

def angle2dcm(yaw, pitch, roll, input_units='rad', rotation_sequence='321'):
    """
    Returns a transformation matrix (aka direction cosine matrix or DCM) which
    transforms from navigation to body frame.  Other names commonly used,
    besides DCM, are `Cbody2nav` or `Rbody2nav`.  The rotation sequence
    specifies the order of rotations when going from navigation-frame to
    body-frame.  The default is '321' (i.e Yaw -> Pitch -> Roll).

    Parameters
    ----------
    yaw   : yaw angle, units of input_units.
    pitch : pitch angle, units of input_units.
    roll  : roll angle , units of input_units.
    input_units: units for input angles {'rad', 'deg'}, optional.
    rotationSequence: assumed rotation sequence {'321', others can be
                                                implemented in the future}.

    Returns
    -------
    Rnav2body: 3x3 transformation matrix (numpy matrix data type).  This can be
               used to convert from navigation-frame (e.g NED) to body frame.

    Notes
    -----
    Since Rnav2body is a proper transformation matrix, the inverse
    transformation is simply the transpose.  Hence, to go from body->nav,
    simply use: Rbody2nav = Rnav2body.T

    Examples:
    ---------
    >>> import numpy as np
    >>> from nav import angle2dcm
    >>> g_ned = np.matrix([[0, 0, 9.8]]).T # gravity vector in NED frame
    >>> yaw, pitch, roll = np.deg2rad([90, 15, 0]) # vehicle orientation
    >>> g_body = Rnav2body * g_ned
    >>> g_body
    matrix([[-2.53642664],
            [ 0.        ],
            [ 9.4660731 ]])

    >>> g_ned_check = Rnav2body.T * g_body
    >>> np.linalg.norm(g_ned_check - g_ned) < 1e-10 # should match g_ned
    True

    Reference
    ---------
    [1] Equation 2.4, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] eul2Cbn.m function (note, this function gives body->nav) at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz
    """
    # Apply necessary unit transformations.
    if input_units == 'rad':
        pass
    elif input_units == 'deg':
        yaw, pitch, roll = np.radians([yaw, pitch, roll])

    # Build transformation matrix Rnav2body.
    s_r, c_r = np.sin(roll) , np.cos(roll)
    s_p, c_p = np.sin(pitch), np.cos(pitch)
    s_y, c_y = np.sin(yaw)  , np.cos(yaw)

    if rotation_sequence == '321':
        # This is equivalent to Rnav2body = R(roll) * R(pitch) * R(yaw)
        # where R() is the single axis rotation matrix.  We implement
        # the expanded form for improved efficiency.
        Rnav2body = np.matrix([
                [c_y*c_p               ,  s_y*c_p              , -s_p    ],
                [-s_y*c_r + c_y*s_p*s_r,  c_y*c_r + s_y*s_p*s_r,  c_p*s_r],
                [ s_y*s_r + c_y*s_p*c_r, -c_y*s_r + s_y*s_p*c_r,  c_p*c_r]])

    else:
        # No other rotation sequence is currently implemented
        print('WARNING (angle2dcm): requested rotation_sequence is unavailable.')
        print('                     NaN returned.')
        Rnav2body = np.nan

    return Rnav2body

def cartesian_to_spherical(cart_array):
    """Convert the Cartesian vector [x, y, z] to spherical coordinates [r, theta, phi].

    The parameter r is the radial distance, theta is the polar angle, and phi is the azimuth.


    @param vector:  The Cartesian vector [x, y, z].
    @type vector:   numpy rank-1, 3D array
    @return:        The spherical coordinate vector [r, theta, phi].
    @rtype:         numpy rank-1, 3D array
    """

    # The radial distance.
    r = np.linalg.norm(cart_array, axis=1)

    # Unit vector.
    unit = cart_array / r[..., None]

    # The polar angle.
    theta = np.arccos(unit[:, 2])

    # The azimuth.
    phi = np.arctan2(unit[:,1], unit[:,0])

    # Return the spherical coordinate vector.
    return np.array([r, theta, phi], np.float64).T


def convert_to_body(elevation, azimuth, roll, pitch, yaw):
    # Convert to cartesian co-ordinates
    sunpos_NED_cart = spherical_to_cartesian(np.array([1., np.radians(elevation), np.radians(azimuth)])[None, ...])

    # Transform to body frame
    RMat = angle2dcm(yaw, pitch, roll, input_units='rad', rotation_sequence='321')
    sunpos_BODY_cart = np.dot(RMat, sunpos_NED_cart.T)

    # Get azimuth and elevation relative to body frame
    r, elevation_BODY, azimuth_BODY = cartesian_to_spherical(sunpos_BODY_cart.T)[0]

    # Conver to degrees
    elevation_BODY = np.degrees(elevation_BODY)
    azimuth_BODY = np.degrees(azimuth_BODY)

    return elevation_BODY, azimuth_BODY


def eval_sunpos_from_stdin():
    """
    Read stdin and evaluate the sun position - output to stdout
    sys.stdin needs to be csv data containing log time, latitude and longitude
    if sys.stdin also contains 3 more entries - roll, pitch, yaw, this function will also output the relative position of the sun from the vehicle
    :return: stdout output
    """

    for line in sys.stdin:
        csv_values = line.split(',')
        timestr, latitude, longitude = csv_values[:3]

        # Evaluate the sun position
        elevation_ned, azimuth_ned = lat_long_to_elaz(float(latitude), float(longitude), logStrToTime(timestr).astype(datetime.datetime))
        output = [elevation_ned, azimuth_ned]

        # Evaluate the relative position of the sun if vehicle position is given
        if len(csv_values) == 6:
            r, p, y = [float(x) for x in csv_values[3:]]
            elevation_body, azimuth_body = convert_to_body(elevation_ned, azimuth_ned, r, p, y)
            output += [elevation_body, azimuth_body]

        # Print to stdout
        print (','.join([str(x) for x in output]))


if __name__ == '__main__':
    # Evalute position using stdin
    eval_sunpos_from_stdin()
