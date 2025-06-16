// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2014, Sony Mobile Communications Inc.
 *
 * based on work by flto at https://github.com/flto/linux/tree/msm8930
 * values from downstream
 */

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/extcon-provider.h>
#include <linux/regulator/driver.h>

#define CHG_BUCK_CLOCK_CTRL		0x14
#define CHG_BUCK_CLOCK_CTRL_8038	0xD

#define CHG_CNTRL		0x204
#define CTRL_EN			BIT(7)

#define PM8921_USB_OTG_CTL	0x348
#define OTG_CTL_EN		BIT(0)
#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07

#define PM8921_CHG_IBAT_MAX	0x205
#define PM8921_DC_IMAX		0x344
#define PM8921_USB_IMAX		0x444
#define PM8921_CHG_IBAT_SAFE	0x210
#define PM8921_CHG_CFG		0x043
#define PM8921_CHG_CTRL		0x049
#define PM8921_CHG_EN_BIT	BIT(7)
#define PM8921_CHG_VDD_MAX	0x220
#define PM8921_CHG_VDD_SAFE	0x221

#define CHG_TEST         	0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F

#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VBAT_BOOT_THRESH	0x222

#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C

#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6
#define OVP_USB_UVD		0x2B7
#define PM8921_USB_TRIM_SEL	0x339
#define TCHG_MAX		BIT(7)

#define CHG_WDOG_TIME		0x062
#define CHG_WDOG_EN		0x065
#define WDOG_EN			BIT(7)

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define CHG_BATFET_ON_BIT	BIT(3)

#define CHG_VCP_EN		BIT(0)
#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
#define CHG_IBAT_TERM_CHG	0x05b

#define IBAT_TERM_CHG_IEOC	BIT(7)
#define IBAT_TERM_CHG_IEOC_BMS	BIT(7)
#define IBAT_TERM_CHG_IEOC_CHG	0

#define PM8921_BUCK_REG_MODE	0x174
#define BUCK_REG_MODE		BIT(0)
#define BUCK_REG_MODE_VBAT	BIT(0)

#define PM8921_CHG_V_MASK	0x7F
#define PM8921_CHG_I_MASK	0x3F
#define PM8921_CHG_ITERM_MASK	0xF

#define MV(x)			(((x) - 3240) / 20)
#define MA(x)			(((x) - 225) / 50)

#define STATUS_USBIN_VALID	BIT(0) /* USB connection is valid */
#define STATUS_DCIN_VALID	BIT(1) /* DC connection is valid */
#define STATUS_BAT_HOT		BIT(2) /* Battery temp 1=Hot, 0=Cold */
#define STATUS_BAT_OK		BIT(3) /* Battery temp OK */
#define STATUS_BAT_PRESENT	BIT(4) /* Battery is present */
#define STATUS_CHG_DONE		BIT(5) /* Charge cycle is complete */
#define STATUS_CHG_TRKL		BIT(6) /* Trickle charging */
#define STATUS_CHG_FAST		BIT(7) /* Fast charging */
#define STATUS_CHG_GONE		BIT(8) /* No charger is connected */

enum pm8921_attr {
	ATTR_BAT_ISAFE,
	ATTR_BAT_IMAX,
	ATTR_BAT_VMAX,
	ATTR_DCIN_IMAX,
	ATTR_USBIN_IMAX,
	_ATTR_CNT,
};

struct pm8921_charger {
	unsigned int addr;
	struct device *dev;
	struct extcon_dev *edev;

	bool dc_disabled;
	unsigned long status;
	struct mutex statlock;

	unsigned int attr[_ATTR_CNT];

	struct power_supply *usb_psy;
	struct power_supply *dc_psy;
	struct power_supply *bat_psy;
	struct regmap *regmap;

	struct regulator_desc otg_rdesc;
	struct regulator_dev *otg_reg;
};

static const unsigned int pm8921_usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_NONE,
};

static int pm8921_vmax_fn(unsigned int index)
{
	return 4500000 + index * 10000;
}

static int pm8921_imax_fn(unsigned int index)
{
	if (index < 2)
		return 100000 + index * 50000;
	return index * 100000;
}

static int pm8921_bat_imax_fn(unsigned int index)
{
	return 225000 + index * 50000;
}

static const struct pm8921_charger_attr {
	const char *name;
	unsigned int reg;
	unsigned int safe_reg;
	unsigned int max;
	unsigned int min;
	unsigned int fail_ok;
	int (*hw_fn)(unsigned int);
} pm8921_charger_attrs[] = {
	[ATTR_BAT_ISAFE] = {
		.name = "qcom,fast-charge-safe-current",
		.reg = PM8921_CHG_IBAT_SAFE,
		.max = 3375000,
		.min = 225000,
		.hw_fn = pm8921_bat_imax_fn,
		.fail_ok = 1,
	},
	[ATTR_BAT_IMAX] = {
		.name = "qcom,fast-charge-current-limit",
		.reg = PM8921_CHG_IBAT_MAX,
		.safe_reg = PM8921_CHG_IBAT_SAFE,
		.max = 3025000,
		.min = 325000,
		.hw_fn = pm8921_bat_imax_fn,
	},
	[ATTR_BAT_VMAX] = {
		.name = "qcom,fast-charge-high-threshold-voltage",
		.reg = PM8921_CHG_VDD_MAX,
		.safe_reg = PM8921_CHG_VDD_SAFE,
		.max = 4500000,
		.min = 3240000,
		.hw_fn = pm8921_vmax_fn,
	},
	[ATTR_DCIN_IMAX] = {
		.name = "qcom,dc-current-limit",
		.reg = PM8921_DC_IMAX,
		.max = 2500000,
		.min = 100000,
		.hw_fn = pm8921_imax_fn,
	},
	[ATTR_USBIN_IMAX] = {
		.name = "usb-charge-current-limit",
		.reg = PM8921_USB_IMAX,
		.max = 2500000,
		.min = 100000,
		.hw_fn = pm8921_imax_fn,
	},
};

static int pm8921_charger_attr_write(struct pm8921_charger *chg,
				     enum pm8921_attr which, unsigned int val)
{
	const struct pm8921_charger_attr *prop;
	int rc;

	prop = &pm8921_charger_attrs[which];

	rc = regmap_write(chg->regmap, chg->addr + prop->reg, val);
	if (rc) {
		pr_err("regmap_write failed: addr=%03X, rc=%d\n",
		       (chg->addr + prop->reg), rc);
		return rc;
	}
	dev_dbg(chg->dev, "%s => %d\n", prop->name, val);

	chg->attr[which] = val;

	return 0;
}

static int pm8921_charger_attr_read(struct pm8921_charger *chg,
				    enum pm8921_attr which)
{
	const struct pm8921_charger_attr *prop;
	unsigned int val;
	int rc;

	prop = &pm8921_charger_attrs[which];

	rc = regmap_read(chg->regmap, chg->addr + prop->reg, &val);
	if (rc) {
		pr_err("regmap_read failed: addr=%03X, rc=%d\n",
		       (chg->addr + prop->reg), rc);
		return rc;
	}
	val = prop->hw_fn(val);
	dev_dbg(chg->dev, "%s => %d\n", prop->name, val);

	chg->attr[which] = val;

	return 0;
}

static int pm8921_charger_attr_parse(struct pm8921_charger *chg,
				     enum pm8921_attr which)
{
	const struct pm8921_charger_attr *prop;
	unsigned int val;
	int rc;

	prop = &pm8921_charger_attrs[which];

	rc = of_property_read_u32(chg->dev->of_node, prop->name, &val);
	if (rc == 0) {
		rc = pm8921_charger_attr_write(chg, which, val);
		if (!rc || !prop->fail_ok)
			return rc;
	}
	return pm8921_charger_attr_read(chg, which);
}

static void pm8921_set_line_flag(struct pm8921_charger *chg, int irq, int flag)
{
	bool state;
	int ret;

	ret = irq_get_irqchip_state(irq, IRQCHIP_STATE_LINE_LEVEL, &state);
	if (ret < 0) {
		dev_err(chg->dev, "failed to read irq line\n");
		return;
	}

	mutex_lock(&chg->statlock);
	if (state)
		chg->status |= flag;
	else
		chg->status &= ~flag;
	mutex_unlock(&chg->statlock);

	dev_dbg(chg->dev, "status = %03lx\n", chg->status);
}

static irqreturn_t pm8921_usb_valid_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_USBIN_VALID);
	extcon_set_state_sync(chg->edev, EXTCON_USB,
			      chg->status & STATUS_USBIN_VALID);
	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_dc_valid_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_DCIN_VALID);
	if (!chg->dc_disabled)
		power_supply_changed(chg->dc_psy);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_bat_temp_handler(int irq, void *_data)
{
	// TODO
	return IRQ_HANDLED;
}

static irqreturn_t pm8921_bat_present_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_BAT_PRESENT);
	power_supply_changed(chg->bat_psy);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_chg_done_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_CHG_DONE);
	power_supply_changed(chg->bat_psy);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_chg_gone_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_CHG_GONE);
	power_supply_changed(chg->bat_psy);
	power_supply_changed(chg->usb_psy);
	if (!chg->dc_disabled)
		power_supply_changed(chg->dc_psy);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_chg_fast_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_CHG_FAST);
	power_supply_changed(chg->bat_psy);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_chg_trkl_handler(int irq, void *_data)
{
	struct pm8921_charger *chg = _data;

	pm8921_set_line_flag(chg, irq, STATUS_CHG_TRKL);
	power_supply_changed(chg->bat_psy);

	return IRQ_HANDLED;
}

static const struct pm8921_irq {
	const char *name;
	irqreturn_t (*handler)(int, void *);
} pm8921_charger_irqs[] = {
	{ "chg-done", pm8921_chg_done_handler },
	{ "chg-fast", pm8921_chg_fast_handler },
	{ "chg-trkl", pm8921_chg_trkl_handler },
	{ "bat-temp-ok", pm8921_bat_temp_handler },
	{ "bat-present", pm8921_bat_present_handler },
	{ "chg-gone", pm8921_chg_gone_handler },
	{ "usb-valid", pm8921_usb_valid_handler },
	{ "dc-valid", pm8921_dc_valid_handler },
};

static int pm8921_usbin_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct pm8921_charger *chg = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		mutex_lock(&chg->statlock);
		val->intval = !(chg->status & STATUS_CHG_GONE) &&
			      (chg->status & STATUS_USBIN_VALID);
		mutex_unlock(&chg->statlock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = 0; //chg->attr[ATTR_USBIN_IMAX];
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = 2500000;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int pm8921_usbin_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	struct pm8921_charger *chg = power_supply_get_drvdata(psy);
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		rc = pm8921_charger_attr_write(chg, ATTR_USBIN_IMAX,
					       val->intval);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int pm8921_dcin_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct pm8921_charger *chg = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		mutex_lock(&chg->statlock);
		val->intval =
			0; // !(chg->status & STATUS_CHG_GONE) && (chg->status & STATUS_DCIN_VALID);
		mutex_unlock(&chg->statlock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = 0; //chg->attr[ATTR_DCIN_IMAX];
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = 2500000;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int pm8921_dcin_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct pm8921_charger *chg = power_supply_get_drvdata(psy);
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		rc = pm8921_charger_attr_write(chg, ATTR_DCIN_IMAX,
					       val->intval);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int pm8921_charger_writable_property(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT;
}

static int pm8921_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct pm8921_charger *chg = power_supply_get_drvdata(psy);
	unsigned long status;
	int rc = 0;

	mutex_lock(&chg->statlock);
	status = chg->status;
	mutex_unlock(&chg->statlock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (status & STATUS_CHG_GONE)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!(status & (STATUS_DCIN_VALID | STATUS_USBIN_VALID)))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (status & STATUS_CHG_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (!(status & STATUS_BAT_OK))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (status & (STATUS_CHG_FAST | STATUS_CHG_TRKL))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else /* everything is ok for charging, but we are not... */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (status & STATUS_BAT_OK)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (status & STATUS_BAT_HOT)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (status & STATUS_CHG_FAST)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else if (status & STATUS_CHG_TRKL)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !!(status & STATUS_BAT_PRESENT);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chg->attr[ATTR_BAT_IMAX];
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chg->attr[ATTR_BAT_VMAX];
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		/* this charger is a single-cell lithium-ion battery charger
		* only.  If you hook up some other technology, there will be
		* fireworks.
		*/
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3000000; /* single-cell li-ion low end */
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int pm8921_battery_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct pm8921_charger *chg = power_supply_get_drvdata(psy);
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = pm8921_charger_attr_write(chg, ATTR_BAT_IMAX, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = pm8921_charger_attr_write(chg, ATTR_BAT_VMAX, val->intval);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int pm8921_battery_writable_property(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property pm8921_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
};

static enum power_supply_property pm8921_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static const struct reg_off_mask_default {
	unsigned int offset;
	unsigned int mask;
	unsigned int value;
	unsigned int rev_mask;
} pm8921_charger_setup[] = {
	{ SYS_CONFIG_2, BOOT_DONE_BIT, BOOT_DONE_BIT },
	{ PM8921_CHG_VDD_SAFE, PM8921_CHG_V_MASK, MV(4200) }, // 4500
	{ CHG_VBAT_DET, PM8921_CHG_V_MASK, MV(4140) },
	{ PM8921_CHG_VDD_MAX, PM8921_CHG_V_MASK, MV(4200) }, // increments?
	{ PM8921_CHG_IBAT_SAFE, PM8921_CHG_I_MASK, MA(1500) },
	{ PM8921_CHG_IBAT_MAX, PM8921_CHG_I_MASK, MA(1100) },
	{ PM8921_CHG_VDD_MAX, PM8921_CHG_ITERM_MASK, (100 - 50) / 10 },

	{ PBL_ACCESS2, ENUM_TIMER_STOP_BIT, ENUM_TIMER_STOP_BIT },
	{ CHG_CNTRL_3, PM8921_CHG_EN_BIT, PM8921_CHG_EN_BIT },

	/* Disable software timer */
	{ CHG_TCHG_MAX, TCHG_MAX, 0 },

	/* Clear and disable watchdog */
	{ CHG_WDOG_TIME, 0xff, 160 },
	{ CHG_WDOG_EN, WDOG_EN, 0 },

	/* Use charger based EoC detection */
	{ CHG_IBAT_TERM_CHG, IBAT_TERM_CHG_IEOC, IBAT_TERM_CHG_IEOC_CHG },

	/* Enable charging */
	{ PM8921_CHG_CTRL, CTRL_EN, CTRL_EN },
};

static char *pm8921_bif[] = { "pm8921-bif" };

static const struct power_supply_desc bat_psy_desc = {
	.name = "pm8921-bif",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pm8921_battery_properties,
	.num_properties = ARRAY_SIZE(pm8921_battery_properties),
	.get_property = pm8921_battery_get_property,
	.set_property = pm8921_battery_set_property,
	.property_is_writeable = pm8921_battery_writable_property,
};

static const struct power_supply_desc usb_psy_desc = {
	.name = "pm8921-usbin",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = pm8921_charger_properties,
	.num_properties = ARRAY_SIZE(pm8921_charger_properties),
	.get_property = pm8921_usbin_get_property,
	.set_property = pm8921_usbin_set_property,
	.property_is_writeable = pm8921_charger_writable_property,
};

static const struct power_supply_desc dc_psy_desc = {
	.name = "pm8921-dcin",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = pm8921_charger_properties,
	.num_properties = ARRAY_SIZE(pm8921_charger_properties),
	.get_property = pm8921_dcin_get_property,
	.set_property = pm8921_dcin_set_property,
	.property_is_writeable = pm8921_charger_writable_property,
};

static int pm8921_chg_otg_enable(struct regulator_dev *rdev)
{
	struct pm8921_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	rc = regmap_update_bits(chg->regmap, chg->addr + PM8921_USB_OTG_CTL,
				OTG_CTL_EN, OTG_CTL_EN);
	if (rc)
		dev_err(chg->dev, "failed to update OTG_CTL\n");
	return rc;
}

static int pm8921_chg_otg_disable(struct regulator_dev *rdev)
{
	struct pm8921_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	rc = regmap_update_bits(chg->regmap, chg->addr + PM8921_USB_OTG_CTL,
				OTG_CTL_EN, 0);
	if (rc)
		dev_err(chg->dev, "failed to update OTG_CTL\n");
	return rc;
}

static int pm8921_chg_otg_is_enabled(struct regulator_dev *rdev)
{
	struct pm8921_charger *chg = rdev_get_drvdata(rdev);
	unsigned int value = 0;
	int rc;

	rc = 0;
	regmap_read(chg->regmap, chg->addr + PM8921_USB_OTG_CTL, &value);
	if (rc)
		dev_err(chg->dev, "failed to read OTG_CTL\n");

	return 0; // !!(value & OTG_CTL_EN);
}

static const struct regulator_ops pm8921_chg_otg_ops = {
	.enable = pm8921_chg_otg_enable,
	.disable = pm8921_chg_otg_disable,
	.is_enabled = pm8921_chg_otg_is_enabled,
};

static int pm8921_charger_probe(struct platform_device *pdev)
{
	struct power_supply_config bat_cfg = {};
	struct power_supply_config usb_cfg = {};
	struct power_supply_config dc_cfg = {};
	struct pm8921_charger *chg;
	struct regulator_config config = {};
	int rc, i;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	mutex_init(&chg->statlock);

	chg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "reg", &chg->addr);
	if (rc) {
		dev_err(&pdev->dev, "missing or invalid 'reg' property\n");
		return rc;
	}

#if 0
	rc = regmap_read(chg->regmap, chg->addr + SMBB_MISC_REV2, &chg->revision);
	if (rc) {
		dev_err(&pdev->dev, "unable to read revision\n");
		return rc;
	}

	chg->revision += 1;
	if (chg->revision != 2 && chg->revision != 3) {
		dev_err(&pdev->dev, "v1 hardware not supported\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "Initializing SMBB rev %u", chg->revision);
#endif

	chg->dc_disabled =
		of_property_read_bool(pdev->dev.of_node, "qcom,disable-dc");

	for (i = 0; i < _ATTR_CNT; ++i) {
		rc = pm8921_charger_attr_parse(chg, i);
		if (rc) {
			dev_err(&pdev->dev, "failed to parse/apply settings\n");
			return rc;
		}
	}

	bat_cfg.drv_data = chg;
	bat_cfg.fwnode = dev_fwnode(&pdev->dev);
	chg->bat_psy =
		devm_power_supply_register(&pdev->dev, &bat_psy_desc, &bat_cfg);
	if (IS_ERR(chg->bat_psy)) {
		dev_err(&pdev->dev, "failed to register battery\n");
		return PTR_ERR(chg->bat_psy);
	}

	usb_cfg.drv_data = chg;
	usb_cfg.supplied_to = pm8921_bif;
	usb_cfg.num_supplicants = ARRAY_SIZE(pm8921_bif);
	chg->usb_psy =
		devm_power_supply_register(&pdev->dev, &usb_psy_desc, &usb_cfg);
	if (IS_ERR(chg->usb_psy)) {
		dev_err(&pdev->dev, "failed to register USB power supply\n");
		return PTR_ERR(chg->usb_psy);
	}

	chg->edev =
		devm_extcon_dev_allocate(&pdev->dev, pm8921_usb_extcon_cable);
	if (IS_ERR(chg->edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	rc = devm_extcon_dev_register(&pdev->dev, chg->edev);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return rc;
	}

	if (!chg->dc_disabled) {
		dc_cfg.drv_data = chg;
		dc_cfg.supplied_to = pm8921_bif;
		dc_cfg.num_supplicants = ARRAY_SIZE(pm8921_bif);
		chg->dc_psy = devm_power_supply_register(&pdev->dev,
							 &dc_psy_desc, &dc_cfg);
		if (IS_ERR(chg->dc_psy)) {
			dev_err(&pdev->dev,
				"failed to register DC power supply\n");
			return PTR_ERR(chg->dc_psy);
		}
	}

	for (i = 0; i < ARRAY_SIZE(pm8921_charger_irqs); ++i) {
		int irq;

		irq = platform_get_irq_byname(pdev,
					      pm8921_charger_irqs[i].name);
		if (irq < 0) {
			dev_err(&pdev->dev, "failed to get irq '%s'\n",
				pm8921_charger_irqs[i].name);
			return irq;
		}

		pm8921_charger_irqs[i].handler(irq, chg);

		rc = devm_request_threaded_irq(
			&pdev->dev, irq, NULL, pm8921_charger_irqs[i].handler,
			IRQF_ONESHOT, pm8921_charger_irqs[i].name, chg);
		if (rc) {
			dev_err(&pdev->dev, "failed to request irq '%s'\n",
				pm8921_charger_irqs[i].name);
			return rc;
		}
	}

	/*
	 * otg regulator is used to control VBUS voltage direction
	 * when USB switches between host and gadget mode
	 */
	chg->otg_rdesc.id = -1;
	chg->otg_rdesc.name = "otg-vbus";
	chg->otg_rdesc.ops = &pm8921_chg_otg_ops;
	chg->otg_rdesc.owner = THIS_MODULE;
	chg->otg_rdesc.type = REGULATOR_VOLTAGE;
	chg->otg_rdesc.supply_name = "usb-otg-in";
	chg->otg_rdesc.of_match = "otg-vbus";

	config.dev = &pdev->dev;
	config.driver_data = chg;

	chg->otg_reg =
		devm_regulator_register(&pdev->dev, &chg->otg_rdesc, &config);
	if (IS_ERR(chg->otg_reg))
		return PTR_ERR(chg->otg_reg);
	for (i = 0; i < ARRAY_SIZE(pm8921_charger_setup); ++i) {
		const struct reg_off_mask_default *r = &pm8921_charger_setup[i];
		rc = regmap_update_bits(chg->regmap, chg->addr + r->offset,
					r->mask, r->value);
		if (rc) {
			dev_err(&pdev->dev,
				"unable to initializing charging, bailing\n");
			return rc;
		}
	}

	platform_set_drvdata(pdev, chg);

	return 0;
}

static void pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_charger *chg;

	chg = platform_get_drvdata(pdev);

	// regmap_update_bits(chg->regmap, chg->addr + CHG_CTRL, CTRL_EN, 0);

	return;
}

static const struct of_device_id pm8921_charger_id_table[] = {
	{ .compatible = "qcom,pm89xx-charger" },
	{}
};
MODULE_DEVICE_TABLE(of, pm8921_charger_id_table);

static struct platform_driver pm8921_charger_driver = {
	.probe	  = pm8921_charger_probe,
	.remove	 = pm8921_charger_remove,
	.driver	 = {
		.name   = "qcom-pm89xx-charger",
		.of_match_table = pm8921_charger_id_table,
	},
};
module_platform_driver(pm8921_charger_driver);

MODULE_DESCRIPTION("Qualcomm PMIC89XX Charger");
MODULE_LICENSE("GPL v2");
