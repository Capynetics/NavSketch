import { useState } from "react";

type SidebarItem = {
  label: string;
  href: string;
};

type SidebarProps = {
  items: SidebarItem[];
};

export default function Sidebar({ items }: SidebarProps) {
  const [collapsed, setCollapsed] = useState(false);

  return (
    <div
      className="bg-dark text-white d-flex flex-column"
      style={{
        width: collapsed ? "70px" : "250px",
        height: "100vh",
        transition: "width 0.3s",
      }}
    >
      {/* Header */}
      <div className="d-flex justify-content-between p-3">
        {!collapsed && <h5 className="mb-0">My App</h5>}

        <button
          className="btn btn-outline-light btn-sm"
          onClick={() => setCollapsed(!collapsed)}
        >
          ☰
        </button>
      </div>

      <hr className="m-0" />

      {/* Menu */}
      <ul className="nav nav-pills flex-column p-2">
        {items.map((item) => (
          <li key={item.href} className="nav-item mb-2">
            <a
              href={item.href}
              className="nav-link text-white"
            >
              {collapsed ? "•" : item.label}
            </a>
          </li>
        ))}
      </ul>
    </div>
  );
}