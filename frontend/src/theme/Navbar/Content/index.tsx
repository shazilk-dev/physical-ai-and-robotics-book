import React from "react";
import NavbarContent from "@theme-original/Navbar/Content";
import AuthButtons from "@site/src/components/Auth/AuthButtons";
import styles from "./styles.module.css";

export default function NavbarContentWrapper(props) {
  return (
    <div className={styles.navbarContentWrapper}>
      <NavbarContent {...props} />
      <div className={styles.authButtonsContainer}>
        <AuthButtons />
      </div>
    </div>
  );
}
