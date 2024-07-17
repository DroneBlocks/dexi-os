source "arm" "ubuntu" {
  file_urls             = ["https://cdimage.ubuntu.com/releases/jammy/release/ubuntu-22.04.4-preinstalled-server-arm64+raspi.img.xz"]
  file_checksum_url     = "https://cdimage.ubuntu.com/releases/jammy/release/SHA256SUMS"
  file_checksum_type    = "sha256"
  file_target_extension = "xz"
  file_unarchive_cmd    = ["xz", "--decompress", "$ARCHIVE_PATH"]
  image_build_method    = "resize"
  image_path            = "dexi_ubuntu_22.img"
  image_size            = "8G"
  image_type            = "dos"
  image_partitions {
    name         = "boot"
    type         = "c"
    start_sector = "2048"
    filesystem   = "fat"
    size         = "256M"
    mountpoint   = "/boot/firmware"
  }
  image_partitions {
    name         = "root"
    type         = "83"
    start_sector = "526336"
    filesystem   = "ext4"
    size         = "0"
    mountpoint   = "/"
  }
  image_chroot_mounts {
    mount_type       = "rbind"
	  source_path      = "/dev"
	  destination_path = "/dev"
  }
  image_chroot_mounts {
    mount_type       = "proc"
	  source_path      = "/proc"
	  destination_path = "/proc"
  }
  image_chroot_mounts {
    mount_type       = "rbind"
	  source_path      = "/sys"
	  destination_path = "/sys"
  }
  image_chroot_mounts {
    mount_type       = "rbind"
	  source_path      = "/var/run/dbus"
	  destination_path = "/var/run/dbus"
  }
  image_chroot_env             = ["PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin"]
  qemu_binary_source_path      = "/usr/bin/qemu-aarch64-static"
  qemu_binary_destination_path = "/usr/bin/qemu-aarch64-static"
}

build {
  sources = ["source.arm.ubuntu"]
  provisioner "file" {
    source = "resources"
    destination = "/tmp/"
  }
  provisioner "shell" {
    script = "resources/provision.sh"
  }
}