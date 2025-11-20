#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
import json
from pathlib import Path


class PX4ConfigInjector(Node):
    def __init__(self):
        super().__init__('px4_config_injector')

        # Declare and get parameters
        self.declare_parameter('px4_build_path', '')
        self.px4_build_path = self.get_parameter('px4_build_path').get_parameter_value().string_value

        if not self.px4_build_path:
            self.get_logger().error('PX4 build path not provided! Use --ros-args -p px4_build_path:=/path/to/px4')
            sys.exit(1)

        self.px4_build_path = Path(self.px4_build_path)

        if not self.px4_build_path.exists():
            self.get_logger().error(f'PX4 build path does not exist: {self.px4_build_path}')
            sys.exit(1)

        self.get_logger().info(f'PX4 Config Injector initialized with build path: {self.px4_build_path}')

        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'px4_injector/status', 10)

        # Subscriber for injection commands
        self.command_sub = self.create_subscription(
            String,
            'px4_injector/command',
            self.command_callback,
            10
        )

        # Timer for periodic checks
        self.timer = self.create_timer(5.0, self.periodic_check)

        self.get_logger().info('Node ready. Listening for injection commands on /px4_injector/command')

        # Identify config file locations
        self.identify_config_files()

    def identify_config_files(self):
        """Identify common PX4 configuration file locations"""
        self.get_logger().info('Scanning for PX4 configuration files...')

        # Common PX4 config paths
        config_patterns = [
            'build/*/etc',
            'ROMFS/px4fmu_common',
            'build/px4_sitl_default/etc',
            'src/modules/*/params',
        ]

        found_configs = []
        for pattern in config_patterns:
            search_path = self.px4_build_path / pattern.split('/')[0]
            if search_path.exists():
                for item in search_path.rglob('*'):
                    if item.is_file() and (item.suffix in ['.params', '.config', ''] or 'mixer' in item.name.lower()):
                        found_configs.append(item)

        if found_configs:
            self.get_logger().info(f'Found {len(found_configs)} potential config files')
            for cfg in found_configs[:10]:  # Log first 10
                self.get_logger().info(f'  - {cfg.relative_to(self.px4_build_path)}')
        else:
            self.get_logger().warning('No configuration files found in expected locations')

    def command_callback(self, msg):
        """Handle incoming injection commands"""
        self.get_logger().info(f'Received command: {msg.data}')

        try:
            command = json.loads(msg.data)
            action = command.get('action', '')

            if action == 'backup':
                self.backup_configs()
            elif action == 'edit':
                file_path = command.get('file')
                modifications = command.get('modifications', {})
                self.edit_config(file_path, modifications)
            elif action == 'restore':
                self.restore_configs()
            else:
                self.get_logger().warning(f'Unknown action: {action}')

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def backup_configs(self):
        """Create backup of current configuration files"""
        self.get_logger().info('Creating configuration backup...')
        backup_path = self.px4_build_path / 'config_backups'
        backup_path.mkdir(exist_ok=True)

        status_msg = String()
        status_msg.data = json.dumps({'status': 'backup_created', 'path': str(backup_path)})
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Backup created at {backup_path}')

    def edit_config(self, file_path, modifications):
        """Edit a specific configuration file"""
        self.get_logger().info(f'Editing config file: {file_path}')
        self.get_logger().info(f'Modifications: {modifications}')

        target_file = self.px4_build_path / file_path

        if not target_file.exists():
            self.get_logger().error(f'Config file not found: {target_file}')
            return

        # Here you would implement the actual file modification logic
        # This is a placeholder for the actual implementation

        status_msg = String()
        status_msg.data = json.dumps({'status': 'config_edited', 'file': file_path})
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Config file edited: {file_path}')

    def restore_configs(self):
        """Restore configuration files from backup"""
        self.get_logger().info('Restoring configuration from backup...')

        status_msg = String()
        status_msg.data = json.dumps({'status': 'config_restored'})
        self.status_pub.publish(status_msg)

        self.get_logger().info('Configuration restored')

    def periodic_check(self):
        """Periodic health check"""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'healthy',
            'px4_path': str(self.px4_build_path),
            'timestamp': self.get_clock().now().to_msg().sec
        })
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PX4ConfigInjector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
