################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
wol_kachel_rev_3.lss \

FLASH_IMAGE += \
wol_kachel_rev_3.hex \

EEPROM_IMAGE += \
wol_kachel_rev_3.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: wol_kachel_rev_3.elf secondary-outputs

# Tool invocations
wol_kachel_rev_3.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,wol_kachel_rev_3.map -mmcu=atmega88 -o "wol_kachel_rev_3.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

wol_kachel_rev_3.lss: wol_kachel_rev_3.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S wol_kachel_rev_3.elf  >"wol_kachel_rev_3.lss"
	@echo 'Finished building: $@'
	@echo ' '

wol_kachel_rev_3.hex: wol_kachel_rev_3.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex wol_kachel_rev_3.elf  "wol_kachel_rev_3.hex"
	@echo 'Finished building: $@'
	@echo ' '

wol_kachel_rev_3.eep: wol_kachel_rev_3.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex wol_kachel_rev_3.elf  "wol_kachel_rev_3.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: wol_kachel_rev_3.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega88 wol_kachel_rev_3.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) wol_kachel_rev_3.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
