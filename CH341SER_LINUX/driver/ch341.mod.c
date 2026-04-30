#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x8e17b3ae, "idr_destroy" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x69acdf38, "memcpy" },
	{ 0xe91d650f, "usb_autopm_get_interface_async" },
	{ 0x30791a20, "usb_anchor_urb" },
	{ 0x622784ac, "usb_autopm_get_interface" },
	{ 0x77a4dae, "usb_control_msg" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0xf2999cd4, "kmalloc_caches" },
	{ 0x4524dc3a, "kmalloc_trace" },
	{ 0x37a0cba, "kfree" },
	{ 0xe7d07011, "usb_ifnum_to_if" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xb8f11603, "idr_alloc" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xf9dea841, "tty_port_init" },
	{ 0x57447872, "usb_alloc_coherent" },
	{ 0xd0a9607b, "usb_alloc_urb" },
	{ 0x7665a95b, "idr_remove" },
	{ 0x4aac5e11, "usb_free_coherent" },
	{ 0x7e9859a3, "_dev_info" },
	{ 0xb2fcc37e, "usb_driver_claim_interface" },
	{ 0x82bf4dba, "usb_get_intf" },
	{ 0xa08e8981, "tty_port_register_device" },
	{ 0x4b25751e, "usb_free_urb" },
	{ 0x6d9e237d, "__tty_insert_flip_string_flags" },
	{ 0xc8a05f50, "tty_flip_buffer_push" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x20978fb9, "idr_find" },
	{ 0x718192ce, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xc6cbbc89, "capable" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x2025b545, "pcpu_hot" },
	{ 0x4afb2238, "add_wait_queue" },
	{ 0x1000e51, "schedule" },
	{ 0x37110088, "remove_wait_queue" },
	{ 0xcd9c13a3, "tty_termios_hw_change" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0xfddb15aa, "usb_put_intf" },
	{ 0x728932fc, "tty_port_tty_get" },
	{ 0xe1c4919d, "tty_vhangup" },
	{ 0xbfb6be4d, "tty_kref_put" },
	{ 0xf935f19a, "tty_unregister_device" },
	{ 0x740a4e0f, "usb_driver_release_interface" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xd2225a53, "usb_submit_urb" },
	{ 0xf0047e36, "_dev_err" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xacb1f78, "usb_autopm_put_interface_async" },
	{ 0x3a0ba475, "usb_kill_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0x47cc71f1, "tty_port_put" },
	{ 0xe418209f, "__tty_alloc_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xea0198e6, "tty_register_driver" },
	{ 0x45e2962, "usb_register_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0x6fc0fa15, "tty_unregister_driver" },
	{ 0xf9098f7f, "tty_driver_kref_put" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0xe2964344, "__wake_up" },
	{ 0x644f0d11, "__dynamic_dev_dbg" },
	{ 0xa72e661f, "tty_port_tty_hangup" },
	{ 0x4a86a22d, "usb_autopm_get_interface_no_resume" },
	{ 0x7c21b7f2, "usb_autopm_put_interface" },
	{ 0xb0a47b6e, "usb_get_from_anchor" },
	{ 0xae6d4553, "tty_port_tty_wakeup" },
	{ 0x119d6f75, "tty_port_hangup" },
	{ 0x6d471aef, "tty_port_close" },
	{ 0x80f511cb, "tty_port_open" },
	{ 0xa93b7332, "usb_deregister" },
	{ 0x708cd699, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v1A86p7523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p7522d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p5523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86pE523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v4348p5523d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "8B035E6078723C06DFB968D");
