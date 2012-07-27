
namespace A {
	class XXX {
		public:
			XXX(int x, float y);
			~XXX();
	};


	class MyClass {
		public:
			class NestedClass {
				public:
					OptimisedSubMeshGeometry() :vertexData(0), indexData(0) {}

					float nestedMethod();

				protected:
					float somenestedprotected();

			};

			virtual void purevirt() = 0;
			int realMethod() {
				return somefunction();
			}
			struct MyStruct : public XXX {
				int someint;
			};
			// forward declarations
			class LODBucket;

	};	// legal to end class with }; ?
}

typedef char mychar;
typedef char mystring[20];

struct {
 unsigned int __w_termsig:7;
 unsigned int __w_coredump:1;
 unsigned int __w_retcode:8;
 unsigned int:16;
}
