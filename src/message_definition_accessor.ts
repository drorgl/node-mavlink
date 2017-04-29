export interface IMavlinkEnumEntryParamAttribute {
	index: string;
}

export interface IMavlinkEnumEntryParam {
	_: string;
	$: IMavlinkEnumEntryParamAttribute;
}

export interface IMavlinkEnumEntryAttribute {
	value: string;
	name: string;
}

export interface IMavlinkEnumEntry {
	$: IMavlinkEnumAttribute;
	description: string[];
	params: IMavlinkEnumEntryParam[];
}

export interface IMavlinkEnumAttribute {
	name: string;
}

export interface IMavlinkEnum {
	$: IMavlinkEnumAttribute;
	description: string[];
	entry: IMavlinkEnumEntry[];
}

export interface IMavlinkEnums {
	enum: IMavlinkEnum[];
}

export interface IMavlinkMessageAttribute {
	id: string;
	name: string;
}

export interface IMavlinkFieldAttribute {
	type: string;
	name: string;
}

export interface IMavlinkField {
	_: string;
	$: IMavlinkFieldAttribute;
}

export interface IMavlinkMessage {
	$: IMavlinkMessageAttribute;
	description: string[];
	field: IMavlinkField[];
}

export interface IMavlinkMessages {
	message: IMavlinkMessage[];
}

export interface IMavlinkDefinition {
	include?: string[];
	version?: string[];
	enums: IMavlinkEnums[];
	messages: IMavlinkMessages[];
}

export interface IMessageDefinitionFile {
	mavlink: IMavlinkDefinition;
}
