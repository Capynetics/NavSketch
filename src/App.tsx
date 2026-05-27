import Sidebar from "./components/sidebar";

function App() {
  return (
    <div className="d-flex">
      <Sidebar
        items={[
          { label: "Dashboard", href: "/" },
          { label: "Users", href: "/users" },
          { label: "Settings", href: "/settings" },
        ]}
      />

      <main className="p-4 flex-grow-1">
        <h1>Main Content</h1>
      </main>
    </div>
  );
}

export default App;