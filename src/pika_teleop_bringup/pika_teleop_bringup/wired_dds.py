"""Select the wired DDS source and avoid duplicate remote locator traffic."""

import ipaddress
import json
from pathlib import Path
import subprocess


def _wired_route(peer):
    ipaddress.IPv4Address(peer)
    result = subprocess.run(
        ['ip', '-j', '-4', 'route', 'get', peer],
        check=True, capture_output=True, text=True,
    )
    routes = json.loads(result.stdout)
    if not routes:
        raise RuntimeError(f'No IPv4 route to the wired DDS peer {peer}')
    interface = routes[0].get('dev', '')
    address = routes[0].get('prefsrc', '')
    device = Path('/sys/class/net') / interface
    if not address or not (device / 'device').exists() or (device / 'wireless').exists():
        raise RuntimeError(
            f'DDS peer {peer} does not route through a physical wired '
            f'IPv4 interface (dev={interface!r}, source={address!r})'
        )
    ipaddress.IPv4Address(address)
    return interface, address


def _profile(address, remote_unicast_locators):
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>pika_wired_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>{address}</address>
      </interfaceWhiteList>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>pika_local_shm</transport_id>
      <type>SHM</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="pika_wired" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>pika_wired_udp</transport_id>
        <transport_id>pika_local_shm</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <allocation>
        <remote_locators>
          <max_unicast_locators>{remote_unicast_locators}</max_unicast_locators>
          <max_multicast_locators>1</max_multicast_locators>
        </remote_locators>
      </allocation>
    </rtps>
  </participant>
</profiles>
'''


def configure_wired_dds(context):
    """Run before any Node/ExecuteProcess so the official stack inherits it."""
    import launch.logging
    from launch.actions import LogInfo, SetEnvironmentVariable
    from launch.substitutions import LaunchConfiguration

    enabled = LaunchConfiguration('dds_wired_only').perform(context).lower()
    if enabled not in ('true', 'false'):
        raise ValueError('dds_wired_only must be true or false')
    if enabled == 'false':
        return [LogInfo(msg='Pika DDS launch profile disabled by launch argument')]
    peer = LaunchConfiguration('dds_wired_peer').perform(context)
    remote_unicast_locators = int(
        LaunchConfiguration('dds_remote_unicast_locators').perform(context)
    )
    if not 1 <= remote_unicast_locators <= 4:
        raise ValueError('dds_remote_unicast_locators must be between 1 and 4')
    interface, address = _wired_route(peer)
    profile_path = Path(launch.logging.launch_config.log_dir) / 'fastdds_wired.xml'
    profile_path.write_text(
        _profile(address, remote_unicast_locators), encoding='utf-8'
    )
    run_info = profile_path.parent / 'run_info.txt'
    with run_info.open('a', encoding='utf-8') as stream:
        stream.write(
            f'dds_wired_peer={peer}\n'
            f'dds_wired_interface={interface}\n'
            f'dds_wired_address={address}\n'
            f'dds_remote_unicast_locators={remote_unicast_locators}\n'
            f'dds_profile={profile_path}\n'
        )
    return [
        SetEnvironmentVariable(
            name='FASTRTPS_DEFAULT_PROFILES_FILE', value=str(profile_path),
        ),
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp'),
        LogInfo(msg=(
            f'Pika DDS: source={interface} ({address}), peer={peer}, '
            f'remote_unicast_locators={remote_unicast_locators}; '
            f'profile={profile_path}'
        )),
    ]
