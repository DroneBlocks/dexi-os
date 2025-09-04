variable "provision_script" {
  description = "Path to the provision script to use"
  type        = string
  default     = "resources_raspberry_pi_os/provision_ark_cm4.sh"
}

source "arm" "raspberry_pi_os_ark_cm4" {
  file_urls             = ["file:///build/base_images/bookwork_jazzy_docker_shrinked.img.gz.xz"]
  file_checksum_type    = "none"  # Skip checksum verification for local files
  file_target_extension = "xz"
  file_unarchive_cmd    = ["xz", "--decompress", "$ARCHIVE_PATH"]
  image_build_method    = "resize"
  image_path            = "dexi_raspberry_pi_os_ark_cm4.img"
  image_size            = "30G"
  image_type            = "dos"
  image_partitions {
    name         = "boot"
    type         = "c"
    start_sector = "8192"
    filesystem   = "fat"
    size         = "256M"
    mountpoint   = "/boot"
  }
  image_partitions {
    name         = "root"
    type         = "83"
    start_sector = "532480"
    filesystem   = "ext4"
    size         = "0"
    mountpoint   = "/"
  }
  image_chroot_env             = ["PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin"]
  qemu_binary_source_path      = "/usr/bin/qemu-aarch64-static"
  qemu_binary_destination_path = "/usr/bin/qemu-aarch64-static"
}

build {
  sources = ["source.arm.raspberry_pi_os_ark_cm4"]
  provisioner "file" {
    source = "resources_raspberry_pi_os"
    destination = "/tmp/resources"
  }
  provisioner "shell" {
    script = var.provision_script
  }
} 