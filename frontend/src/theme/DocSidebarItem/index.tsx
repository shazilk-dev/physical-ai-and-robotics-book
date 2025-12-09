import React, { type ReactElement } from "react";
import DocSidebarItem from "@theme-original/DocSidebarItem";
import type { Props } from "@theme/DocSidebarItem";

export default function DocSidebarItemWrapper(props: Props): ReactElement {
  return <DocSidebarItem {...props} />;
}
