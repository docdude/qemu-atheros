block/block-backend.c:    qemu_opts_del(dinfo->opts);
blockdev.c:        if (dinfo->type == type) {
blockdev.c:    dinfo->auto_del = 1;
blockdev.c:    if (dinfo && dinfo->auto_del) {
blockdev.c:        if (dinfo && dinfo->type == type
blockdev.c:            && dinfo->bus == bus && dinfo->unit == unit) {
blockdev.c:        if (!blk_get_attached_dev(blk) && !dinfo->is_default &&
blockdev.c:            dinfo->type != IF_NONE) {
blockdev.c:            qemu_opts_loc_restore(dinfo->opts);
blockdev.c:                         if_name[dinfo->type], dinfo->bus, dinfo->unit);
blockdev.c:        if (dinfo && dinfo->type == type && dinfo->bus > max_bus) {
blockdev.c:            max_bus = dinfo->bus;
blockdev.c:    dinfo->opts = all_opts;
blockdev.c:    dinfo->type = type;
blockdev.c:    dinfo->bus = bus_id;
blockdev.c:    dinfo->unit = unit_id;
blockdev.c:        dinfo->media_cd = media == MEDIA_CDROM;
device-hotplug.c:    switch (dinfo->type) {
device-hotplug.c:        monitor_printf(mon, "Can't hot-add drive to type %d\n", dinfo->type);
hw/sparc64/niagara.c:            dinfo->is_default = 1;
hw/scsi/scsi-disk.c:    if (dinfo && dinfo->media_cd) {
hw/scsi/scsi-bus.c:        qemu_opts_loc_restore(dinfo->opts);
hw/mips/mips_malta.c:               blk_name(dinfo->bdrv), fl_sectors);
hw/mips/mips_ar71xx.c:               blk_name(dinfo->bdrv), fl_sectors);
hw/arm/tosa.c:    if (!dinfo || dinfo->media_cd)
hw/arm/spitz.c:    if (!dinfo || dinfo->media_cd)
hw/core/qdev-properties-system.c:        if (dinfo && dinfo->type != IF_NONE) {
hw/ide/qdev.c:    ide_dev_initfn(dev, dinfo && dinfo->media_cd ? IDE_CD : IDE_HD, errp);
monitor.c:    fdinfo->fdset_id = mon_fdset->id;
monitor.c:    fdinfo->fd = mon_fdset_fd->fd;
roms/openbios/fs/hfsplus/hfsp_record.c:    printf(  "frRect              :\t");    record_print_Rect(&dinfo->frRect);
roms/openbios/fs/hfsplus/hfsp_record.c:    printf("\nfrFlags             :\t0X%X\n",    dinfo->frFlags);
roms/openbios/fs/hfsplus/hfsp_record.c:    printf(  "frLocation          :\t");    record_print_Point(&dinfo->frLocation);
roms/openbios/fs/hfsplus/hfsp_record.c:    printf("\nfrView              :\t0X%X\n",    dinfo->frView);
roms/u-boot-sam460ex/drivers/mtd/nand/ndfc.c:	struct nand_chip *this = mtdinfo->priv;
roms/u-boot-sam460ex/drivers/mtd/nand/ndfc.c:	struct nand_chip *this = mtdinfo->priv;
roms/u-boot-sam460ex/drivers/mtd/nand/ndfc.c:	struct nand_chip *this = mtdinfo->priv;
roms/u-boot-sam460ex/drivers/mtd/nand/ndfc.c:	struct nand_chip *this = mtdinfo->priv;
roms/u-boot-sam460ex/drivers/mtd/nand/ndfc.c:	struct nand_chip *this = mtdinfo->priv;
roms/u-boot-sam460ex/drivers/mtd/nand/ndfc.c:	struct nand_chip *this = mtdinfo->priv;
roms/u-boot/board/cssi/MCR3000/nand.c:	struct nand_chip *this	= mtdinfo->priv;
vl.c:    dinfo->is_default = true;
