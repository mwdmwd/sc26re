/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

struct controller_report;

int lizard_init(void);
void lizard_update(const struct controller_report *report);
void lizard_transport_reset(void);
void lizard_clear_digital_mappings(void);
void lizard_set_default_digital_mappings(void);
