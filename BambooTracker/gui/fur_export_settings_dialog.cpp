/*
 * Copyright (C) 2026 BambooTracker contributors
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "fur_export_settings_dialog.hpp"
#include "ui_fur_export_settings_dialog.h"

FurExportSettingsDialog::FurExportSettingsDialog(QWidget *parent)
	: QDialog(parent),
	  ui(new Ui::FurExportSettingsDialog)
{
	ui->setupUi(this);

	setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

	for (QRadioButton *button : { ui->ym2608RadioButton, ui->ym2610bRadioButton })
		connect(button, &QAbstractButton::toggled,
				this, &FurExportSettingsDialog::updateSupportInformation);

	updateSupportInformation();
}

FurExportSettingsDialog::~FurExportSettingsDialog()
{
	delete ui;
}

io::FurExportTarget FurExportSettingsDialog::getExportTarget() const
{
	return ui->ym2610bRadioButton->isChecked() ? io::FurExportTarget::YM2610B : io::FurExportTarget::YM2608;
}

void FurExportSettingsDialog::updateSupportInformation()
{
	switch (getExportTarget()) {
	case io::FurExportTarget::YM2608:
		ui->rhythmLabel->setText(tr("YM2608 rhythm channels"));
		ui->adpcmLabel->setText(tr("ADPCM channel"));
		break;
	case io::FurExportTarget::YM2610B:
		ui->rhythmLabel->setText(tr("ADPCM-A channels (built-in rhythm samples are embedded)"));
		ui->adpcmLabel->setText(tr("ADPCM-B channel"));
		break;
	}
}
