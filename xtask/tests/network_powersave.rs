use std::path::Path;

/// The initial migration has two successful paths: creating `robot-wifi`, and returning after a
/// reboot when NetworkManager already owns wlan0. Both must repair the saved profile, or this fix
/// works only on freshly flashed boards and not on the board that exposed the failure.
#[test]
fn migration_persists_and_applies_the_wifi_power_policy() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let script = std::fs::read_to_string(root.join("scripts/migrate-network.sh")).unwrap();

    let function = script
        .split("disable_wifi_power_saving() {")
        .nth(1)
        .and_then(|tail| tail.split("\n}").next())
        .expect("migrate-network.sh must define disable_wifi_power_saving");
    assert!(
        function.contains("802-11-wireless.powersave 2"),
        "the saved robot-wifi profile must explicitly disable power saving"
    );
    assert!(
        function.contains("iw dev wlan0 set power_save off"),
        "the running association must be updated without cycling the SSH link"
    );

    let profile = script
        .split("migrate_wifi_profile() {")
        .nth(1)
        .and_then(|tail| tail.split("\n}").next())
        .expect("migrate-network.sh must define migrate_wifi_profile");
    assert_eq!(
        profile.matches("disable_wifi_power_saving").count(),
        2,
        "both an existing profile and a newly created profile need the policy"
    );

    let already_owned = script
        .split("if nm_owns_wifi; then")
        .nth(1)
        .and_then(|tail| tail.split("retire_net_check").next())
        .expect("main must handle the already-migrated path");
    assert!(
        already_owned.contains("disable_wifi_power_saving"),
        "the post-reboot run must repair profiles made by older provisioning code"
    );
}
